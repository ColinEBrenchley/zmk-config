#pragma once

#include <stdint.h>
#include <lvgl.h>

struct six_by_six_sprite {
    const uint8_t *data;
    uint16_t width;
    uint16_t height;
    uint16_t duration_ms;
};

extern const struct six_by_six_sprite six_by_six_sprites[];
extern const uint16_t six_by_six_sprite_count;
extern const uint16_t six_by_six_sprite_width;
extern const uint16_t six_by_six_sprite_height;

extern const lv_img_dsc_t *six_by_six_sprite_images[];