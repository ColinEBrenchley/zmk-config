#define DT_DRV_COMPAT zmk_behavior_led_fx

#include <zephyr/device.h>
#include <zephyr/logging/log.h>

#include <drivers/behavior.h>

#include "led_fx.h"

LOG_MODULE_DECLARE(led_fx, CONFIG_ZMK_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

#ifndef DEVICE_API
#define DEVICE_API(_class, _name) const struct _class##_driver_api _name
#endif

static int on_led_fx_pressed(struct zmk_behavior_binding *binding,
                             struct zmk_behavior_binding_event event) {
    int rc = led_fx_command(binding->param1);
    if (rc) {
        LOG_ERR("Unknown LED FX command %d", binding->param1);
    }
    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_led_fx_released(struct zmk_behavior_binding *binding,
                              struct zmk_behavior_binding_event event) {
    return ZMK_BEHAVIOR_OPAQUE;
}

static DEVICE_API(behavior, led_fx_driver_api) = {
    .binding_pressed = on_led_fx_pressed,
    .binding_released = on_led_fx_released,
};

#define LED_FX_INST(n)                                                                             \
    BEHAVIOR_DT_INST_DEFINE(n, NULL, NULL, NULL, NULL, POST_KERNEL,                                \
                            CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &led_fx_driver_api);

DT_INST_FOREACH_STATUS_OKAY(LED_FX_INST)

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
