/*******************************************************************************
 * Waveshare ESP32-S3-Touch-LCD-7 LVGL Bouncing Balls
 *
 * A resolution-independent demo: 1..20 circular balls bounce under gravity,
 * each a random color, re-launching to a random height on every floor bounce.
 * Two sliders at the bottom select the number of balls and the speed.
 *
 * The whole UI is laid out from the live display resolution at runtime (see
 * ui.cpp), so this sketch's ui.cpp / ui.h drop unchanged into any board whose
 * lv_setup.hpp follows the same shape, in any orientation.
 *
 * Requires the following libraries:
 *   - chipguy_ESP32S3_Touch_LCD_7_display  (display + touch, this library)
 *   - chipguy_ESP32-S3-Touch-LCD-7_board   (shared I2C bus + expander; a dependency)
 *   - lvgl 9
 *
 * Arduino IDE Board Settings:
 *   - Board: ESP32S3 Dev Module
 *   - PSRAM: OPI PSRAM
 *   - Upload Speed: 460800
 *   - Flash Size: 8 MegaBytes (64 megabits)
 *   - USB CDC On Boot: Enabled
 *
 * Copyright (c) 2025 chipguyhere
 * MIT License
 ******************************************************************************/

// The bundled lv_conf.h is based on LVGL 9.3 (LVGL 9 compatible).
#include "lv_conf.h"
#include "lvgl.h"
#include "lv_setup.hpp"

// The bouncing-balls UI lives in ui.h / ui.cpp in this sketch folder.
#include "ui.h"

void setup() {
    // Note: USB serial output on this device is Serial0,
    // and "USB CDC On Boot" must be enabled to see anything
    Serial0.begin(115200);

    // Initialize display, touch, and LVGL.
    // Rotation: lv_setup.begin(180) for an upside-down panel, or begin(90) /
    // begin(270) for portrait (480x800).  The demo adapts to whatever resolution
    // the chosen orientation reports.
    lv_setup.begin();
    Serial0.printf("LVGL initialized with %dx%d touchscreen\n",
                   display.width(), display.height());

    // Build the bouncing-balls screen.
    ui_init();
}

void loop() {
    // Give loop control to LVGL.
    lv_timer_handler();
    delay(5);
}
