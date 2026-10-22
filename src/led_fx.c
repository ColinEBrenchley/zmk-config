/*
 * Custom WS2812 effects: selectable red/green palette, alert, chase pulse,
 * breathing, scanner, and one-shot sweep/dark-wave overlays.
 *
 * Drives the strip behind the `zmk,underglow` chosen node directly, so
 * CONFIG_ZMK_RGB_UNDERGLOW must be disabled.
 */

#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>

#if IS_ENABLED(CONFIG_RAW_HID)
#include <raw_hid/events.h>
#endif

#include "led_fx.h"

LOG_MODULE_REGISTER(led_fx, CONFIG_ZMK_LOG_LEVEL);

#define STRIP_NODE DT_CHOSEN(zmk_underglow)
#define NUM_LEDS DT_PROP(STRIP_NODE, chain_length)

#define TICK_MS 30
#define BRI_STEP 5
#define BRI_MIN 5
#define TAIL 5 /* head + 4 trailing LEDs */
#define ANIMATION_LED_COUNT 8
#define RED_FLASH_TICKS 8
#define SOUND_TICKS 36
#define METEOR_TAIL 6
#define METEOR_PAUSE 12 /* ticks between meteors */

enum fx_mode {
    MODE_SOLID,
    MODE_ALERT,
    MODE_CHASE,
    MODE_BREATHE,
    MODE_SCANNER,
    MODE_OFF,
    MODE_STARFIELD,
    MODE_METEOR,
    MODE_COUNT,
};

/* Follow LEDs 4,3,2,1,5,6,7,8. LEDs 9-12 stay white. */
static const uint8_t animation_path[ANIMATION_LED_COUNT] = {
    3, 2, 1, 0, 4, 5, 6, 7,
};
static const uint8_t static_white_leds[] = {8, 9, 10, 11};

static const struct device *strip = DEVICE_DT_GET(STRIP_NODE);
static struct led_rgb px[NUM_LEDS];
static struct k_work_delayable tick_work;

static volatile uint8_t mode = MODE_SOLID;
static volatile bool green = false;
static volatile uint8_t bri = CONFIG_LED_FX_DEFAULT_BRIGHTNESS; /* percent */
static volatile int32_t sweep_t = -1;                           /* -1 = inactive */
static volatile int32_t wave_t = -1;                            /* -1 = inactive */
static volatile uint8_t red_flash_ticks;
static volatile int32_t sound_t = -1;                           /* -1 = inactive */
static uint32_t ticks;

static const uint8_t trail[TAIL] = {255, 110, 48, 20, 8};
static const uint8_t sound_wave[16] = {
    8, 20, 48, 115, 255, 115, 48, 16, 4, 12, 32, 80, 180, 80, 28, 8,
};

static inline uint8_t sc(uint8_t v) { return ((uint16_t)v * bri) / 100; }

static inline uint8_t color_r(uint8_t level) {
    return green ? 0 : sc(level);
}

static inline uint8_t color_g(uint8_t level) {
    return green ? sc(level) : 0;
}

static inline void set_px(int i, uint8_t r, uint8_t g, uint8_t b) {
    if (i < 0 || i >= NUM_LEDS) {
        return;
    }
    px[i].r = r;
    px[i].g = g;
    px[i].b = b;
}

static inline void set_path_px(int position, uint8_t r, uint8_t g, uint8_t b) {
    if (position >= 0 && position < ANIMATION_LED_COUNT) {
        set_px(animation_path[position], r, g, b);
    }
}

static inline void max_px(int i, uint8_t r, uint8_t g, uint8_t b) {
    if (i < 0 || i >= NUM_LEDS) {
        return;
    }
    px[i].r = MAX(px[i].r, r);
    px[i].g = MAX(px[i].g, g);
    px[i].b = MAX(px[i].b, b);
}

static inline void max_path_px(int position, uint8_t r, uint8_t g, uint8_t b) {
    if (position >= 0 && position < ANIMATION_LED_COUNT) {
        max_px(animation_path[position], r, g, b);
    }
}

static inline uint8_t sat_add(uint8_t a, uint8_t b) {
    uint16_t s = (uint16_t)a + b;
    return s > 255 ? 255 : s;
}

static inline void add_px(int i, uint8_t r, uint8_t g, uint8_t b) {
    if (i < 0 || i >= NUM_LEDS) {
        return;
    }
    px[i].r = sat_add(px[i].r, r);
    px[i].g = sat_add(px[i].g, g);
    px[i].b = sat_add(px[i].b, b);
}

static inline void add_path_px(int position, uint8_t r, uint8_t g, uint8_t b) {
    if (position >= 0 && position < ANIMATION_LED_COUNT) {
        add_px(animation_path[position], r, g, b);
    }
}

static void render_static_white_leds(void) {
    uint8_t brightness = sc(255);
    for (size_t i = 0; i < ARRAY_SIZE(static_white_leds); i++) {
        set_px(static_white_leds[i], brightness, brightness, brightness);
    }
}

static void render(void) {
    memset(px, 0, sizeof(px));

    bool sound_active = sound_t >= 0 && mode != MODE_OFF;
    if (sound_active) {
        uint8_t background = sc(24);
        for (int i = 0; i < NUM_LEDS; i++) {
            set_px(i, 0, 0, background);
        }
    } else {
        switch (mode) {
        case MODE_SOLID:
            for (int i = 0; i < NUM_LEDS; i++) {
                set_px(i, color_r(255), color_g(255), 0);
            }
            break;

        case MODE_ALERT: {
            bool on = ((ticks / 8) % 2) == 0; /* ~240 ms on / 240 ms off */
            for (int i = 0; i < NUM_LEDS; i++) {
                if (on) {
                    set_px(i, color_r(255), color_g(255), 0);
                } else {
                    set_px(i, color_r(255) / 8, color_g(255) / 8, 0);
                }
            }
            break;
        }

        case MODE_CHASE: {
            int head = (ticks / 3) % (ANIMATION_LED_COUNT + TAIL); /* ~90 ms per LED */
            for (int i = 0; i < ANIMATION_LED_COUNT; i++) {
                set_path_px(i, color_r(255) / 5, color_g(255) / 5, 0);
            }
            for (int t = 0; t < TAIL; t++) {
                uint8_t lvl = sc(trail[t]);
                max_path_px(head - t, color_r(trail[t]), color_g(trail[t]),
                            t == 0 ? lvl / 2 : 0);
            }
            break;
        }

        case MODE_BREATHE: {
            uint32_t p = (ticks * 2) % 128;
            uint32_t tri = p < 64 ? p * 4 : (127 - p) * 4;
            uint32_t v = (tri * tri) / 255;
            v = 10 + (v * 245) / 255;
            for (int i = 0; i < NUM_LEDS; i++) {
                set_px(i, color_r((uint8_t)v), color_g((uint8_t)v), 0);
            }
            break;
        }

        case MODE_SCANNER: {
            int span = ANIMATION_LED_COUNT - 1;
            int period = 2 * span;
            int pos = (ticks / 2) % period;
            if (pos > span) {
                pos = period - pos;
            }
            for (int i = 0; i < ANIMATION_LED_COUNT; i++) {
                set_path_px(i, color_r(255) / 16, color_g(255) / 16, 0);
            }
            max_path_px(pos, color_r(255), color_g(255), 0);
            max_path_px(pos - 1, color_r(80), color_g(80), 0);
            max_path_px(pos + 1, color_r(80), color_g(80), 0);
            max_path_px(pos - 2, color_r(16), color_g(16), 0);
            max_path_px(pos + 2, color_r(16), color_g(16), 0);
            break;
        }

        case MODE_STARFIELD: {
            for (int i = 0; i < NUM_LEDS; i++) {
                set_px(i, sc(10), sc(10), sc(10));
            }
            for (int i = 0; i < 5; i++) {
                uint32_t h = (uint32_t)(ticks * 7 + i * 131) * 0x9E3779B9u;
                h ^= h >> 16;
                int pos = (int)(h % NUM_LEDS);
                uint32_t v = h % 100;
                if (v < 25) {
                    uint8_t b = sc(255);
                    max_px(pos, b, b, b);
                } else if (v < 60) {
                    uint8_t b = sc(120);
                    max_px(pos, b, b, b);
                }
            }
            break;
        }

        case MODE_METEOR: {
            int span = ANIMATION_LED_COUNT - 1;
            int travel = span + METEOR_TAIL;
            int cycle = travel + METEOR_PAUSE;
            int pos = ticks % cycle;
            if (pos < travel) {
                bool forward = ((ticks / cycle) % 2) == 0;
                int head = forward ? pos : (cycle - 1 - pos);
                for (int t = 0; t < METEOR_TAIL; t++) {
                    int idx = forward ? head - t : head + t;
                    if (idx < 0 || idx > span) {
                        break;
                    }
                    uint8_t lvl = sc(trail[t]);
                    max_path_px(idx, color_r(trail[t]), color_g(trail[t]),
                                t == 0 ? lvl / 2 : 0);
                }
            }
            break;
        }

        default: /* MODE_OFF */
            break;
        }
    }

    /* One-shot bright pulse in the selected color, 2 ticks (~60 ms) per LED. */
    if (!sound_active && sweep_t >= 0 && mode != MODE_OFF) {
        int head = sweep_t / 2;
        for (int t = 0; t < TAIL; t++) {
            uint8_t lvl = sc(trail[t]);
            add_path_px(head - t, color_r(trail[t]), color_g(trail[t]),
                        t == 0 ? lvl / 2 : 0);
        }
        sweep_t++;
        if ((sweep_t / 2) - (TAIL - 1) >= ANIMATION_LED_COUNT) {
            sweep_t = -1;
        }
    } else if (mode == MODE_OFF) {
        sweep_t = -1;
    }

    /* One-shot dark pulse: each LED goes dark briefly, then resumes its mode. */
    if (!sound_active && wave_t >= 0 && mode != MODE_OFF) {
        set_path_px(wave_t / 2, 0, 0, 0);
        wave_t++;
        if (wave_t / 2 >= ANIMATION_LED_COUNT) {
            wave_t = -1;
        }
    } else if (mode == MODE_OFF) {
        wave_t = -1;
    }

    /* Short cyan soundwave with irregular white twinkles. */
    if (sound_t >= 0 && mode != MODE_OFF) {
        for (int position = 0; position < ANIMATION_LED_COUNT; position++) {
            uint8_t level = sc(sound_wave[(sound_t + position * 2) % ARRAY_SIZE(sound_wave)]);
            max_path_px(position, level / 4, (level * 3) / 4, level);

            uint32_t sparkle = (uint32_t)(sound_t + 1) * 0x9E3779B9u;
            sparkle ^= (uint32_t)(position + 1) * 0x85EBCA6Bu;
            sparkle ^= sparkle >> 16;
            if ((sparkle & 0x1Fu) < 3) {
                uint8_t brightness = sc(255);
                max_path_px(position, brightness, brightness, brightness);
            }
        }
        sound_t++;
        if (sound_t >= SOUND_TICKS) {
            sound_t = -1;
        }
    } else if (mode == MODE_OFF) {
        sound_t = -1;
    }

    if (!sound_active && mode != MODE_STARFIELD) {
        render_static_white_leds();
    }
    if (red_flash_ticks > 0) {
        uint8_t brightness = red_flash_ticks > 5 ? 255 : red_flash_ticks * 40;
        for (int i = 0; i < NUM_LEDS; i++) {
            set_px(i, brightness, 0, 0);
        }
        red_flash_ticks--;
    }
}

static void tick(struct k_work *work) {
    ticks++;
    render();

    int rc = led_strip_update_rgb(strip, px, NUM_LEDS);
    if (rc) {
        LOG_ERR("led_strip_update_rgb failed: %d", rc);
    }

    k_work_reschedule(&tick_work, K_MSEC(TICK_MS));
}

void led_fx_trigger_sweep(void) {
    sweep_t = 0;
    k_work_reschedule(&tick_work, K_NO_WAIT);
}

int led_fx_command(uint32_t cmd) {
    switch (cmd) {
    case LED_FX_CMD_CHASE:
    case LED_FX_CMD_BREATHE:
    case LED_FX_CMD_SCANNER:
    case LED_FX_CMD_OFF:
        mode = (uint8_t)cmd; /* enum fx_mode values match the command ids */
        ticks = 0;
        break;

    case LED_FX_CMD_RED:
        green = false;
        mode = MODE_SOLID;
        ticks = 0;
        break;

    case LED_FX_CMD_ALERT:
        /* Toggle alert without changing the selected color. */
        mode = (mode == MODE_ALERT) ? MODE_SOLID : MODE_ALERT;
        ticks = 0;
        break;

    case LED_FX_CMD_SWEEP:
        led_fx_trigger_sweep();
        return 0;

    case LED_FX_CMD_WAVE:
        wave_t = 0;
        k_work_reschedule(&tick_work, K_NO_WAIT);
        return 0;

    case LED_FX_CMD_FLASH:
        red_flash_ticks = RED_FLASH_TICKS;
        k_work_reschedule(&tick_work, K_NO_WAIT);
        return 0;

    case LED_FX_CMD_SOUND:
        sound_t = 0;
        k_work_reschedule(&tick_work, K_NO_WAIT);
        return 0;

    case LED_FX_CMD_NEXT:
        mode = (mode + 1) % MODE_COUNT;
        ticks = 0;
        break;

    case LED_FX_CMD_PREV:
        mode = (mode + MODE_COUNT - 1) % MODE_COUNT;
        ticks = 0;
        break;

    case LED_FX_CMD_STARFIELD:
        mode = MODE_STARFIELD;
        ticks = 0;
        break;

    case LED_FX_CMD_METEOR:
        mode = MODE_METEOR;
        ticks = 0;
        break;

    case LED_FX_CMD_GREEN:
        green = !green;
        mode = MODE_SOLID;
        ticks = 0;
        break;

    case LED_FX_CMD_BRI_UP:
        bri = MIN(bri + BRI_STEP, CONFIG_LED_FX_MAX_BRIGHTNESS);
        break;

    case LED_FX_CMD_BRI_DN:
        bri = (bri > BRI_MIN + BRI_STEP) ? bri - BRI_STEP : BRI_MIN;
        break;

    default:
        return -EINVAL;
    }

    k_work_reschedule(&tick_work, K_NO_WAIT); /* show the change immediately */
    return 0;
}

#if IS_ENABLED(CONFIG_LED_FX_SWEEP_ON_KEYPRESS)
static int led_fx_position_listener(const zmk_event_t *eh) {
    const struct zmk_position_state_changed *ev = as_zmk_position_state_changed(eh);
    if (ev != NULL && ev->state) {
        led_fx_trigger_sweep();
    }
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(led_fx, led_fx_position_listener);
ZMK_SUBSCRIPTION(led_fx, zmk_position_state_changed);
#endif

#if IS_ENABLED(CONFIG_RAW_HID)
static int led_fx_raw_hid_listener(const zmk_event_t *eh) {
    const struct raw_hid_received_event *event = as_raw_hid_received_event(eh);
    if (event == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    if (event->data == NULL || event->length < 1) {
        LOG_WRN("Ignoring empty Raw HID LED FX report");
        return ZMK_EV_EVENT_BUBBLE;
    }

    int rc = led_fx_command(event->data[0]);
    if (rc) {
        LOG_WRN("Ignoring unknown Raw HID LED FX command %u", event->data[0]);
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(led_fx_raw_hid, led_fx_raw_hid_listener);
ZMK_SUBSCRIPTION(led_fx_raw_hid, raw_hid_received_event);
#endif

static int led_fx_init(void) {
    if (!device_is_ready(strip)) {
        LOG_ERR("LED strip device not ready");
        return -ENODEV;
    }

    if (bri > CONFIG_LED_FX_MAX_BRIGHTNESS) {
        bri = CONFIG_LED_FX_MAX_BRIGHTNESS;
    }

    k_work_init_delayable(&tick_work, tick);
    k_work_schedule(&tick_work, K_MSEC(100));
    return 0;
}

SYS_INIT(led_fx_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
