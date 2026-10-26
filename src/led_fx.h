#pragma once

#include <stdint.h>

/* Keep in sync with the FX_* defines in starwars_panel_behaviors.dtsi */
enum led_fx_cmd {
    LED_FX_CMD_RED = 0,
    LED_FX_CMD_ALERT = 1,
    LED_FX_CMD_CHASE = 2,
    LED_FX_CMD_BREATHE = 3,
    LED_FX_CMD_SCANNER = 4,
    LED_FX_CMD_OFF = 5,
    LED_FX_CMD_SWEEP = 6,
    LED_FX_CMD_NEXT = 7,
    LED_FX_CMD_BRI_UP = 8,
    LED_FX_CMD_BRI_DN = 9,
    LED_FX_CMD_WAVE = 10,
    LED_FX_CMD_GREEN = 12,
    LED_FX_CMD_PREV = 13,
    LED_FX_CMD_FLASH = 14,
    LED_FX_CMD_SOUND = 15,
    LED_FX_CMD_STARFIELD = 16,
    LED_FX_CMD_METEOR = 17,
    LED_FX_CMD_RIPPLE = 18,
    LED_FX_CMD_DUAL_CHASE = 19,
    LED_FX_CMD_RAINBOW = 20,
    LED_FX_CMD_EQUALIZER = 21,
    LED_FX_CMD_RAIN = 22,
    LED_FX_CMD_SABER_IGNITE_BLUE = 23,
    LED_FX_CMD_SABER_IGNITE_RED = 24,
    LED_FX_CMD_SABER_DEACTIVATE = 25,
    /* Palette colors (also switch to the solid effect). */
    LED_FX_CMD_BLUE = 26,
    LED_FX_CMD_PURPLE = 27,
    LED_FX_CMD_WHITE = 28,
    LED_FX_CMD_YELLOW = 29,
    LED_FX_CMD_ORANGE = 30,
    LED_FX_CMD_CYAN = 31,
    LED_FX_CMD_PINK = 32,
    /* Cycle the palette color without leaving the current effect. */
    LED_FX_CMD_COLOR_NEXT = 33,
    /* Saber ignition colors. */
    LED_FX_CMD_SABER_IGNITE_GREEN = 34,
    LED_FX_CMD_SABER_IGNITE_PURPLE = 35,
    LED_FX_CMD_SABER_IGNITE_WHITE = 36,
    LED_FX_CMD_SABER_IGNITE_YELLOW = 37,
    LED_FX_CMD_SABER_IGNITE_ORANGE = 38,
    LED_FX_CMD_SABER_IGNITE_CYAN = 39,
    LED_FX_CMD_SABER_IGNITE_PINK = 40,
    /* One-shot full-strip flashes in a specific color. FX_FLASH (14) is red. */
    LED_FX_CMD_FLASH_GREEN = 41,
    LED_FX_CMD_FLASH_BLUE = 42,
    LED_FX_CMD_FLASH_PURPLE = 43,
    LED_FX_CMD_FLASH_WHITE = 44,
    LED_FX_CMD_FLASH_YELLOW = 45,
    LED_FX_CMD_FLASH_ORANGE = 46,
    LED_FX_CMD_FLASH_CYAN = 47,
    LED_FX_CMD_FLASH_PINK = 48,
    /* Flash the blade's current saber color (falls back to red when idle). */
    LED_FX_CMD_FLASH_SABER = 49,
    /* Cycle the independent four-LED red light bar. */
    LED_FX_CMD_BAR_NEXT = 50,
    LED_FX_CMD_BAR_PREV = 51,
};

int led_fx_command(uint32_t cmd);
void led_fx_trigger_sweep(void);
