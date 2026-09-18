cmake_minimum_required(VERSION 3.20.0)

if(CONFIG_ZMK_DISPLAY)

    target_sources(
        app
        PRIVATE
            display/six_by_six_display.c
            generated/sprites.c
    )

endif()