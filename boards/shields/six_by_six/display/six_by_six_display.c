#include <lvgl.h>

#include <zmk/display/status_screen.h>

lv_obj_t *zmk_display_status_screen(void) {
    lv_obj_t *screen = lv_obj_create(NULL);

    lv_obj_set_style_bg_color(
        screen,
        lv_color_white(),
        LV_PART_MAIN
    );

    lv_obj_set_style_bg_opa(
        screen,
        LV_OPA_COVER,
        LV_PART_MAIN
    );

    lv_obj_t *box = lv_obj_create(screen);

    lv_obj_set_size(
        box,
        200,
        200
    );

    lv_obj_set_style_bg_color(
        box,
        lv_color_black(),
        LV_PART_MAIN
    );

    lv_obj_set_style_bg_opa(
        box,
        LV_OPA_COVER,
        LV_PART_MAIN
    );

    lv_obj_set_style_border_width(
        box,
        0,
        LV_PART_MAIN
    );

    lv_obj_set_style_radius(
        box,
        0,
        LV_PART_MAIN
    );

    lv_obj_align(
        box,
        LV_ALIGN_CENTER,
        0,
        0
    );

    return screen;
}