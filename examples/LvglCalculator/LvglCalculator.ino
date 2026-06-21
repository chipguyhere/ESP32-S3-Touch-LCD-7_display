/*******************************************************************************
 * Waveshare ESP32-S3-Touch-LCD-7 LVGL Calculator
 *
 * For the Waveshare ESP32-S3-Touch-LCD-7 (800x480 ST7262 RGB LCD
 * + GT911 capacitive touch).
 *
 * A working four-function calculator with a touch keypad, styled after the
 * familiar dark iOS calculator: a black background, dark-gray number keys, a
 * light-gray top function row (backspace / C / %) and an orange operator
 * column (/ x - + =).
 *
 * Resolution independence is the point of this example.  The whole UI is laid
 * out from the live display resolution in ui.cpp — button diameter, gaps,
 * margins and font sizes are all derived from width()/height() at runtime.
 * Nothing is hard-coded to 800x480, so the identical ui.cpp lays out a
 * proportionate keypad on a different panel without edits.  See the layout
 * math in build_keypad() in ui.cpp.  (The calculator is adapted from the
 * rectangular ESP32-S3-Touch-LCD-2 version, since its grid suits a rectangle.)
 *
 * Requires the following libraries:
 *   - chipguy_ESP32S3_Touch_LCD_7_display  (display + touch, this library)
 *   - chipguy_ESP32-S3-Touch-LCD-7_board   (shared I2C bus + expander; a dependency)
 *   - lvgl 9
 *
 * Arduino IDE Board Settings (Tools menu):
 *   - Board: "ESP32S3 Dev Module" (in Tools > Board > esp32)
 *   - Flash Size: "8MB (64Mb)"
 *   - PSRAM: "OPI PSRAM"   (the board has 8MB PSRAM)
 *   - USB CDC On Boot: "Enabled" (so serial works over the native USB port)
 *
 ******************************************************************************/

// The bundled lv_conf.h is based on LVGL 9.3 (LVGL 9 compatible).
// It's important for the config and library versions to be compatible.
#include "lv_conf.h"
#include "lvgl.h"
#include "lv_setup.hpp"

// The calculator UI lives in ui.h / ui.cpp in this sketch folder.
#include "ui.h"

void setup() {
    // Note: USB serial output on this device is Serial0,
    // and "USB CDC On Boot" must be enabled to see anything
    Serial0.begin(115200);

    // Initialize display, touch, and LVGL.  The UI adapts to whatever
    // resolution the display reports.
    lv_setup.begin();
    Serial0.printf("LVGL initialized with %dx%d touchscreen\n",
                   display.width(), display.height());

    // Build the calculator screen.
    ui_init();
}

void loop() {
    // Give loop control to LVGL.
    lv_timer_handler();
    delay(5);
}
