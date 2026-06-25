/*******************************************************************************
 * Waveshare ESP32-S3-Touch-LCD-7 LVGL 9.3 Claude Code Stub
 *
 * A minimal starting point for building an LVGL application from scratch on
 * the Waveshare ESP32-S3-Touch-LCD-7 development board (ST7262 800x480 RGB LCD +
 * GT911 capacitive touch).
 *
 * This is the same scaffold as the Lvgl93SquarelineLauncher example, but with
 * the SquareLine-generated UI removed.  Instead of designing the interface
 * visually and exporting it, you write the UI yourself: by hand, or by handing
 * this sketch to an AI coding assistant such as Claude Code.
 *
 * The UI lives in a tiny ui.h / ui.cpp pair in this sketch folder.  All of the
 * display, touch, and LVGL plumbing is already wired up (see lv_setup.hpp)
 * before ui_init() is called, so ui_init() is a blank canvas: it receives a
 * live LVGL screen and you create widgets on it.  To start your app, open
 * ui.cpp and replace the contents of ui_init() with your own widgets.
 *
 * Because everything except the UI is already wired up, this scaffold is ideal
 * to hand to Claude Code: point it at this sketch folder and describe the app
 * you want.  It can focus entirely on ui.cpp and the screens you add, without
 * touching the display/touch/LVGL plumbing.
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

// This is built for LVGL 9.3, the same version SquareLine generates for.
// It's important for the config and library versions to be compatible.
#include "lv_conf.h"
#include "lvgl.h"
#include "lv_setup.hpp"


// Fonts
// The Montserrat font is built into LVGL in multiple sizes, each size takes up memory.
// We enable the 14 point font as a default font.
#define LV_FONT_MONTSERRAT_14 1
// need other sizes?  Edit lv_conf.h where they are turned off (0) and turn them on (1)

// The UI for this example lives in ui.h / ui.cpp in this sketch folder.
// It is a hand-written "Hello, world!" screen meant as a starting point.
// Build your own application by editing ui_init() in ui.cpp.
#include "ui.h"


void setup() {
    // Note: USB serial output on this device is Serial0,
    // and "USB CDC On Boot" must be enabled to see anything
    Serial0.begin(115200);

    // Initialize display, touch, and LVGL.
    // Rotation: lv_setup.begin(180) for an upside-down panel, or begin(90) /
    // begin(270) for portrait (480x800).  See the README.
    lv_setup.begin();
    Serial0.printf("LVGL initialized with %dx%d touchscreen\n", display.width(), display.height());

    // Start the application's own setup
    ui_init();
}


void loop() {
    // Give loop control to LVGL objects created by the application
    lv_timer_handler(); /* let the GUI do its work */
    delay(5);
}
