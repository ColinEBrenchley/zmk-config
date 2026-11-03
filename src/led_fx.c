/*
 * Custom WS2812 effects: selectable multi-color palette, saber ignition/flicker
 * in several blade colors, alert, chase pulse, breathing, scanner, and one-shot
 * sweep/dark-wave overlays.
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
#define SABER_LED_COUNT 10
#define SABER_TICK_MS 35
#define SABER_FADE_TICKS 20
#define FLASH_TICKS 8
#define SOUND_TICKS 36
#define METEOR_TAIL 6
#define METEOR_PAUSE 12 /* ticks between meteors */
#define RAIN_DROPLETS 4

enum fx_mode {
    MODE_SOLID,
    MODE_ALERT,
    MODE_CHASE,
    MODE_BREATHE,
    MODE_SCANNER,
    MODE_OFF,
    MODE_STARFIELD,
    MODE_METEOR,
    MODE_RIPPLE,
    MODE_DUAL_CHASE,
    MODE_RAINBOW,
    MODE_EQUALIZER,
    MODE_RAIN,
    MODE_COUNT,
};

enum saber_state {
    SABER_IDLE,
    SABER_IGNITING,
    SABER_ON,
    SABER_DEACTIVATING,
    SABER_RESTORING,
};

/* Colors available to the effects and the saber. Keep in sync with the
 * LED_FX_CMD_* color commands in led_fx.h. */
enum led_color {
    LED_COLOR_RED,
    LED_COLOR_GREEN,
    LED_COLOR_BLUE,
    LED_COLOR_PURPLE,
    LED_COLOR_WHITE,
    LED_COLOR_YELLOW,
    LED_COLOR_ORANGE,
    LED_COLOR_CYAN,
    LED_COLOR_PINK,
    LED_COLOR_COUNT,
};

struct led_rgb8 {
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

/* Full-intensity colors used by the non-saber effects. */
static const struct led_rgb8 palette_colors[LED_COLOR_COUNT] = {
    [LED_COLOR_RED] = {255, 0, 0},
    [LED_COLOR_GREEN] = {0, 255, 0},
    [LED_COLOR_BLUE] = {0, 30, 255},
    [LED_COLOR_PURPLE] = {160, 0, 255},
    [LED_COLOR_WHITE] = {255, 255, 255},
    [LED_COLOR_YELLOW] = {255, 220, 0},
    [LED_COLOR_ORANGE] = {255, 90, 0},
    [LED_COLOR_CYAN] = {0, 220, 255},
    [LED_COLOR_PINK] = {255, 40, 160},
};

/* Blade colors. Kept slightly below the palette maxima so the saber keeps the
 * same visual intensity as the original red/blue implementation. */
static const struct led_rgb8 saber_colors[LED_COLOR_COUNT] = {
    [LED_COLOR_RED] = {200, 0, 0},
    [LED_COLOR_GREEN] = {0, 210, 20},
    [LED_COLOR_BLUE] = {0, 20, 210},
    [LED_COLOR_PURPLE] = {130, 0, 210},
    [LED_COLOR_WHITE] = {210, 210, 210},
    [LED_COLOR_YELLOW] = {210, 190, 0},
    [LED_COLOR_ORANGE] = {220, 90, 0},
    [LED_COLOR_CYAN] = {0, 190, 210},
    [LED_COLOR_PINK] = {220, 40, 160},
};

/* Follow LEDs 4,3,2,1,5,6,7,8. LEDs 9-12 stay white. */
static const uint8_t animation_path[ANIMATION_LED_COUNT] = {
    3, 2, 1, 0, 7, 6, 5, 4,
};
static const uint8_t saber_path[SABER_LED_COUNT] = {
    3, 2, 1, 0, 9, 8, 7, 6, 5, 4,
};
static const uint8_t static_white_leds[] = {8, 9, 10, 11};

static const struct device *strip = DEVICE_DT_GET(STRIP_NODE);
static struct led_rgb px[NUM_LEDS];
static struct k_work_delayable tick_work;

static volatile uint8_t mode = MODE_SOLID;
static volatile uint8_t palette = LED_COLOR_RED;
static volatile uint8_t bri = CONFIG_LED_FX_DEFAULT_BRIGHTNESS; /* percent */
static volatile int32_t sweep_t = -1;                           /* -1 = inactive */
static volatile int32_t wave_t = -1;                            /* -1 = inactive */
static volatile uint8_t flash_ticks;
static volatile uint8_t flash_color = LED_COLOR_RED;
static volatile int32_t sound_t = -1;                           /* -1 = inactive */
static uint32_t ticks;
static volatile enum saber_state saber_state;
static volatile uint8_t saber_progress;
static volatile uint8_t saber_color = LED_COLOR_BLUE;
static uint32_t saber_previous_ticks;

static const uint8_t trail[TAIL] = {255, 110, 48, 20, 8};
static const uint8_t sound_wave[16] = {
    8, 20, 48, 115, 255, 115, 48, 16, 4, 12, 32, 80, 180, 80, 28, 8,
};

static inline uint8_t sc(uint8_t v) { return ((uint16_t)v * bri) / 100; }

static inline uint8_t scale_channel(uint8_t channel, uint8_t level) {
    return sc((uint8_t)(((uint16_t)channel * level) / 255));
}

static inline uint8_t color_r(uint8_t level) {
    return scale_channel(palette_colors[palette].r, level);
}

static inline uint8_t color_g(uint8_t level) {
    return scale_channel(palette_colors[palette].g, level);
}

static inline uint8_t color_b(uint8_t level) {
    return scale_channel(palette_colors[palette].b, level);
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

/* Convert a 0-255 hue to RGB (full saturation, full value). */
static inline void hue_to_rgb(uint8_t h, uint8_t *r, uint8_t *g, uint8_t *b) {
    /* 6-segment hue wheel (each 1/6 = 42.67) */
    uint8_t sector = h / 43; /* 0..5 */
    uint8_t rem = (h - sector * 43) * 6; /* 0..255 within segment */

    switch (sector) {
    case 0: *r = 255; *g = rem;   *b = 0;   break;
    case 1: *r = 255 - rem; *g = 255; *b = 0;   break;
    case 2: *r = 0;   *g = 255; *b = rem;   break;
    case 3: *r = 0;   *g = 255 - rem; *b = 255; break;
    case 4: *r = rem; *g = 0;   *b = 255; break;
    default: *r = 255; *g = 0;   *b = 255 - rem; break;
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

/* Helpers for the common "draw the selected palette color at this level" case. */
static inline void set_color_px(int i, uint8_t level) {
    set_px(i, color_r(level), color_g(level), color_b(level));
}

static inline void set_path_color(int position, uint8_t level) {
    if (position >= 0 && position < ANIMATION_LED_COUNT) {
        set_color_px(animation_path[position], level);
    }
}

static inline void max_color_px(int i, uint8_t level) {
    max_px(i, color_r(level), color_g(level), color_b(level));
}

static inline void max_path_color(int position, uint8_t level) {
    if (position >= 0 && position < ANIMATION_LED_COUNT) {
        max_color_px(animation_path[position], level);
    }
}

static inline void add_color_px(int i, uint8_t level) {
    add_px(i, color_r(level), color_g(level), color_b(level));
}

static inline void add_path_color(int position, uint8_t level) {
    if (position >= 0 && position < ANIMATION_LED_COUNT) {
        add_color_px(animation_path[position], level);
    }
}

static void render_static_white_leds(void) {
    uint8_t brightness = sc(255);
    for (size_t i = 0; i < ARRAY_SIZE(static_white_leds); i++) {
        set_px(static_white_leds[i], brightness, brightness, brightness);
    }
}

static void render_saber(void) {
    uint8_t red_level = saber_colors[saber_color].r;
    uint8_t green_level = saber_colors[saber_color].g;
    uint8_t blue_level = saber_colors[saber_color].b;
    int lit_leds = saber_progress;
    bool flicker_active = saber_state == SABER_ON;

    for (int position = 0; position < lit_leds; position++) {
        uint8_t intensity = 255;
        if (flicker_active) {
            uint32_t flicker = (ticks + 1) * 0x9E3779B9u;
            flicker ^= (uint32_t)(position + 1) * 0x85EBCA6Bu;
            flicker ^= flicker >> 16;

            intensity = 175 + (flicker % 81);
            if ((flicker & 0x3Fu) == 0) {
                intensity = 120;
            }
        }

        set_px(saber_path[position], sc((uint8_t)((uint16_t)red_level * intensity / 255)),
               sc((uint8_t)((uint16_t)green_level * intensity / 255)),
               sc((uint8_t)((uint16_t)blue_level * intensity / 255)));
    }
}

static void render(void) {
    memset(px, 0, sizeof(px));

    bool saber_active = saber_state == SABER_IGNITING || saber_state == SABER_ON ||
                        saber_state == SABER_DEACTIVATING;
    bool sound_active = sound_t >= 0 && mode != MODE_OFF && !saber_active;
    if (sound_active) {
        uint8_t background = sc(24);
        for (int i = 0; i < NUM_LEDS; i++) {
            set_px(i, 0, 0, background);
        }
    } else if (saber_active) {
        render_saber();
    } else {
        switch (mode) {
        case MODE_SOLID:
            for (int i = 0; i < NUM_LEDS; i++) {
                set_color_px(i, 255);
            }
            break;

        case MODE_ALERT: {
            bool on = ((ticks / 8) % 2) == 0; /* ~240 ms on / 240 ms off */
            for (int i = 0; i < NUM_LEDS; i++) {
                if (on) {
                    set_color_px(i, 255);
                } else {
                    set_color_px(i, 31);
                }
            }
            break;
        }

        case MODE_CHASE: {
            int head = (ticks / 3) % (ANIMATION_LED_COUNT + TAIL); /* ~90 ms per LED */
            for (int i = 0; i < ANIMATION_LED_COUNT; i++) {
                set_path_color(i, 51);
            }
            for (int t = 0; t < TAIL; t++) {
                max_path_color(head - t, trail[t]);
            }
            break;
        }

        case MODE_BREATHE: {
            uint32_t p = (ticks * 2) % 128;
            uint32_t tri = p < 64 ? p * 4 : (127 - p) * 4;
            uint32_t v = (tri * tri) / 255;
            v = 10 + (v * 245) / 255;
            for (int i = 0; i < NUM_LEDS; i++) {
                set_color_px(i, (uint8_t)v);
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
                set_path_color(i, 16);
            }
            max_path_color(pos, 255);
            max_path_color(pos - 1, 80);
            max_path_color(pos + 1, 80);
            max_path_color(pos - 2, 16);
            max_path_color(pos + 2, 16);
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
                    max_path_color(idx, trail[t]);
                }
            }
            break;
        }

        case MODE_RIPPLE: {
            /* Two pulses expand outward from the center of the path. */
            int span = ANIMATION_LED_COUNT - 1;
            int center = span / 2; /* index 3 */
            int period = span + 1; /* time to reach both edges */
            int dist = (ticks / 2) % (period * 2);
            if (dist > span) {
                dist = span * 2 - dist; /* bounce back for a smooth loop */
            }
            /* Faint base */
            for (int i = 0; i < ANIMATION_LED_COUNT; i++) {
                set_path_color(i, 31);
            }
            /* Bright pulse at distance `dist` from center (both sides) */
            for (int d = 0; d <= dist; d++) {
                int brightness = 255 - d * 40;
                if (brightness < 0) brightness = 0;
                int left = center - d;
                int right = center + d;
                if (left >= 0) {
                    max_path_color(left, (uint8_t)brightness);
                }
                if (right <= span) {
                    max_path_color(right, (uint8_t)brightness);
                }
            }
            break;
        }

        case MODE_DUAL_CHASE: {
            /* Two pulses moving in opposite directions simultaneously. */
            int period = ANIMATION_LED_COUNT + TAIL;
            int head_a = (ticks / 3) % period;
            int head_b = period - (ticks / 3) % period;

            for (int i = 0; i < ANIMATION_LED_COUNT; i++) {
                set_path_color(i, 51);
            }
            for (int t = 0; t < TAIL; t++) {
                /* Head A moving forward */
                max_path_color(head_a - t, trail[t]);
                /* Head B moving backward */
                max_path_color(head_b - t, trail[t]);
            }
            break;
        }

        case MODE_RAINBOW: {
            /* Full hue wheel sweeping along the path over time. */
            uint8_t hue_shift = (uint8_t)(ticks / 2);
            for (int i = 0; i < ANIMATION_LED_COUNT; i++) {
                uint8_t hue = (hue_shift + i * (256 / ANIMATION_LED_COUNT)) & 0xFF;
                uint8_t r, g, b;
                hue_to_rgb(hue, &r, &g, &b);
                set_path_px(i, sc(r), sc(g), sc(b));
            }
            break;
        }

        case MODE_EQUALIZER: {
            /* Pseudo-random bar heights, like a spectrum analyzer. */
            for (int i = 0; i < ANIMATION_LED_COUNT; i++) {
                uint32_t h = (uint32_t)(ticks * 5 + i * 97) * 0x9E3779B9u;
                h ^= h >> 13;
                h ^= h << 11;
                uint8_t level = (uint8_t)(h % 256);
                max_path_color(i, level);
            }
            break;
        }

        case MODE_RAIN: {
            /* Multiple independent droplets with short trails. */
            for (int i = 0; i < ANIMATION_LED_COUNT; i++) {
                set_path_color(i, 25);
            }
            for (int d = 0; d < RAIN_DROPLETS; d++) {
                /* Each droplet has a different speed and offset */
                uint32_t speed = 2 + (d % 3); /* 2, 3, or 4 ticks per step */
                uint32_t offset = d * 17;
                int head = ((ticks / speed) + offset) % (ANIMATION_LED_COUNT + 3);
                for (int t = 0; t < 3; t++) {
                    max_path_color(head - t, trail[t]);
                }
            }
            break;
        }

        default: /* MODE_OFF */
            break;
        }
    }

    /* One-shot bright pulse in the selected color, 2 ticks (~60 ms) per LED. */
    if (!saber_active && !sound_active && sweep_t >= 0 && mode != MODE_OFF) {
        int head = sweep_t / 2;
        for (int t = 0; t < TAIL; t++) {
            add_path_color(head - t, trail[t]);
        }
        sweep_t++;
        if ((sweep_t / 2) - (TAIL - 1) >= ANIMATION_LED_COUNT) {
            sweep_t = -1;
        }
    } else if (mode == MODE_OFF) {
        sweep_t = -1;
    }

    /* One-shot dark pulse: each LED goes dark briefly, then resumes its mode. */
    if (!saber_active && !sound_active && wave_t >= 0 && mode != MODE_OFF) {
        set_path_px(wave_t / 2, 0, 0, 0);
        wave_t++;
        if (wave_t / 2 >= ANIMATION_LED_COUNT) {
            wave_t = -1;
        }
    } else if (mode == MODE_OFF) {
        wave_t = -1;
    }

    /* Short cyan soundwave with irregular white twinkles. */
    if (!saber_active && sound_t >= 0 && mode != MODE_OFF) {
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

    if (!saber_active && !sound_active && mode != MODE_STARFIELD && mode != MODE_RAINBOW) {
        render_static_white_leds();
    }
    if (flash_ticks > 0) {
        uint8_t brightness = flash_ticks > 5 ? 255 : flash_ticks * 40;
        /* The flash used to ignore the brightness setting and drive the strip
         * at 100% for eight ticks. Full-brightness white (all three channels
         * across 12 LEDs) exceeds a USB port's 500 mA budget, sags the 5V rail,
         * and makes the blue channel droop, so the flash looks yellow or the
         * board resets. Cap the white-equivalent current, but let single- and
         * two-channel colors run brighter because they draw less. */
        uint32_t channel_sum = (uint32_t)palette_colors[flash_color].r +
                               palette_colors[flash_color].g +
                               palette_colors[flash_color].b;
        uint32_t flash_bri = channel_sum == 0
                                 ? CONFIG_LED_FX_FLASH_MAX_BRIGHTNESS
                                 : (uint32_t)CONFIG_LED_FX_FLASH_MAX_BRIGHTNESS * 765 /
                                       channel_sum;
        flash_bri = MIN(flash_bri, 100);
        uint8_t r = (uint32_t)palette_colors[flash_color].r * flash_bri * brightness / 25500;
        uint8_t g = (uint32_t)palette_colors[flash_color].g * flash_bri * brightness / 25500;
        uint8_t b = (uint32_t)palette_colors[flash_color].b * flash_bri * brightness / 25500;
        for (int i = 0; i < NUM_LEDS; i++) {
            set_px(i, r, g, b);
        }
        flash_ticks--;
    }

    if (saber_state == SABER_RESTORING) {
        uint16_t brightness = (uint16_t)saber_progress * 255 / SABER_FADE_TICKS;
        for (int i = 0; i < NUM_LEDS; i++) {
            px[i].r = (uint16_t)px[i].r * brightness / 255;
            px[i].g = (uint16_t)px[i].g * brightness / 255;
            px[i].b = (uint16_t)px[i].b * brightness / 255;
        }
    }
}

static void tick(struct k_work *work) {
    ticks++;
    switch (saber_state) {
    case SABER_IGNITING:
        if (++saber_progress >= SABER_LED_COUNT) {
            saber_state = SABER_ON;
        }
        break;
    case SABER_DEACTIVATING:
        if (saber_progress > 0 && --saber_progress == 0) {
            saber_state = SABER_RESTORING;
            ticks = saber_previous_ticks;
        }
        break;
    case SABER_RESTORING:
        if (++saber_progress >= SABER_FADE_TICKS) {
            saber_state = SABER_IDLE;
        }
        break;
    case SABER_IDLE:
    case SABER_ON:
        break;
    }
    render();

    int rc = led_strip_update_rgb(strip, px, NUM_LEDS);
    if (rc) {
        LOG_ERR("led_strip_update_rgb failed: %d", rc);
    }

    uint32_t interval = (saber_state == SABER_IGNITING || saber_state == SABER_DEACTIVATING)
                            ? SABER_TICK_MS
                            : TICK_MS;
    k_work_reschedule(&tick_work, K_MSEC(interval));
}

void led_fx_trigger_sweep(void) {
    sweep_t = 0;
    k_work_reschedule(&tick_work, K_NO_WAIT);
}

static void trigger_flash(uint8_t color) {
    flash_color = color;
    flash_ticks = FLASH_TICKS;
    k_work_reschedule(&tick_work, K_NO_WAIT);
}

static void select_mode(uint8_t next_mode) {
    saber_state = SABER_IDLE;
    mode = next_mode;
    ticks = 0;
}

static void select_palette(uint8_t color) {
    palette = color;
    select_mode(MODE_SOLID);
}

static void ignite_saber(uint8_t color) {
    if (saber_state == SABER_IDLE || saber_state == SABER_RESTORING) {
        saber_previous_ticks = ticks;
    }
    saber_color = color;
    saber_state = SABER_IGNITING;
    saber_progress = 0;
    sweep_t = -1;
    wave_t = -1;
    sound_t = -1;
    flash_ticks = 0;
}

int led_fx_command(uint32_t cmd) {
    switch (cmd) {
    case LED_FX_CMD_CHASE:
    case LED_FX_CMD_BREATHE:
    case LED_FX_CMD_SCANNER:
    case LED_FX_CMD_OFF:
        select_mode((uint8_t)cmd); /* enum fx_mode values match the command ids */
        break;

    case LED_FX_CMD_RED:
        select_palette(LED_COLOR_RED);
        break;

    case LED_FX_CMD_GREEN:
        select_palette(LED_COLOR_GREEN);
        break;

    case LED_FX_CMD_BLUE:
        select_palette(LED_COLOR_BLUE);
        break;

    case LED_FX_CMD_PURPLE:
        select_palette(LED_COLOR_PURPLE);
        break;

    case LED_FX_CMD_WHITE:
        select_palette(LED_COLOR_WHITE);
        break;

    case LED_FX_CMD_YELLOW:
        select_palette(LED_COLOR_YELLOW);
        break;

    case LED_FX_CMD_ORANGE:
        select_palette(LED_COLOR_ORANGE);
        break;

    case LED_FX_CMD_CYAN:
        select_palette(LED_COLOR_CYAN);
        break;

    case LED_FX_CMD_PINK:
        select_palette(LED_COLOR_PINK);
        break;

    case LED_FX_CMD_COLOR_NEXT:
        palette = (palette + 1) % LED_COLOR_COUNT;
        if (mode == MODE_OFF) {
            select_mode(MODE_SOLID);
        }
        break;

    case LED_FX_CMD_ALERT:
        /* Toggle alert without changing the selected color. */
        select_mode((mode == MODE_ALERT) ? MODE_SOLID : MODE_ALERT);
        break;

    case LED_FX_CMD_SWEEP:
        led_fx_trigger_sweep();
        return 0;

    case LED_FX_CMD_WAVE:
        wave_t = 0;
        k_work_reschedule(&tick_work, K_NO_WAIT);
        return 0;

    case LED_FX_CMD_FLASH:
        trigger_flash(LED_COLOR_RED);
        return 0;

    case LED_FX_CMD_FLASH_GREEN:
        trigger_flash(LED_COLOR_GREEN);
        return 0;

    case LED_FX_CMD_FLASH_BLUE:
        trigger_flash(LED_COLOR_BLUE);
        return 0;

    case LED_FX_CMD_FLASH_PURPLE:
        trigger_flash(LED_COLOR_PURPLE);
        return 0;

    case LED_FX_CMD_FLASH_WHITE:
        trigger_flash(LED_COLOR_WHITE);
        return 0;

    case LED_FX_CMD_FLASH_YELLOW:
        trigger_flash(LED_COLOR_YELLOW);
        return 0;

    case LED_FX_CMD_FLASH_ORANGE:
        trigger_flash(LED_COLOR_ORANGE);
        return 0;

    case LED_FX_CMD_FLASH_CYAN:
        trigger_flash(LED_COLOR_CYAN);
        return 0;

    case LED_FX_CMD_FLASH_PINK:
        trigger_flash(LED_COLOR_PINK);
        return 0;

    case LED_FX_CMD_FLASH_SABER:
        /* Use the blade's current color while it is igniting, lit, or fading
         * out; fall back to red when no saber is active. */
        trigger_flash(saber_state == SABER_IDLE ? LED_COLOR_RED : saber_color);
        return 0;

    case LED_FX_CMD_SOUND:
        sound_t = 0;
        k_work_reschedule(&tick_work, K_NO_WAIT);
        return 0;

    case LED_FX_CMD_NEXT:
        select_mode((mode + 1) % MODE_COUNT);
        break;

    case LED_FX_CMD_PREV:
        select_mode((mode + MODE_COUNT - 1) % MODE_COUNT);
        break;

    case LED_FX_CMD_STARFIELD:
        select_mode(MODE_STARFIELD);
        break;

    case LED_FX_CMD_METEOR:
        select_mode(MODE_METEOR);
        break;

    case LED_FX_CMD_RIPPLE:
        select_mode(MODE_RIPPLE);
        break;

    case LED_FX_CMD_DUAL_CHASE:
        select_mode(MODE_DUAL_CHASE);
        break;

    case LED_FX_CMD_RAINBOW:
        select_mode(MODE_RAINBOW);
        break;

    case LED_FX_CMD_EQUALIZER:
        select_mode(MODE_EQUALIZER);
        break;

    case LED_FX_CMD_RAIN:
        select_mode(MODE_RAIN);
        break;

    case LED_FX_CMD_SABER_IGNITE_BLUE:
        ignite_saber(LED_COLOR_BLUE);
        break;

    case LED_FX_CMD_SABER_IGNITE_RED:
        ignite_saber(LED_COLOR_RED);
        break;

    case LED_FX_CMD_SABER_IGNITE_GREEN:
        ignite_saber(LED_COLOR_GREEN);
        break;

    case LED_FX_CMD_SABER_IGNITE_PURPLE:
        ignite_saber(LED_COLOR_PURPLE);
        break;

    case LED_FX_CMD_SABER_IGNITE_WHITE:
        ignite_saber(LED_COLOR_WHITE);
        break;

    case LED_FX_CMD_SABER_IGNITE_YELLOW:
        ignite_saber(LED_COLOR_YELLOW);
        break;

    case LED_FX_CMD_SABER_IGNITE_ORANGE:
        ignite_saber(LED_COLOR_ORANGE);
        break;

    case LED_FX_CMD_SABER_IGNITE_CYAN:
        ignite_saber(LED_COLOR_CYAN);
        break;

    case LED_FX_CMD_SABER_IGNITE_PINK:
        ignite_saber(LED_COLOR_PINK);
        break;

    case LED_FX_CMD_SABER_DEACTIVATE:
        if (saber_state == SABER_IGNITING || saber_state == SABER_ON) {
            if (saber_progress == 0) {
                saber_state = SABER_RESTORING;
                saber_progress = 0;
                ticks = saber_previous_ticks;
            } else {
                saber_state = SABER_DEACTIVATING;
            }
        }
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
