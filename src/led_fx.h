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
};

int led_fx_command(uint32_t cmd);
void led_fx_trigger_sweep(void);
