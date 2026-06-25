#pragma once

// LVGL display and touch driver for the Waveshare ESP32-S3-Touch-LCD-7

#include <ESP32S3_Touch_LCD_7_Board.h>            // shared I2C bus + expander
#include <chipguy_ESP32S3_Touch_LCD_7_display.h>
#include <chipguy_ESP32S3_Touch_LCD_7_touch.h>
#include <chipguy_ESP32S3_Touch_LCD_7_rotate.hpp> // 90/270 rotate + reconcile
#include <Wire.h>
#include <lvgl.h>
#include "esp_heap_caps.h"

// The board owns the shared I2C bus and the CH422G expander (display reset/
// backlight and touch reset).  The display and touch drivers reach their
// hardware through it.
static ESP32S3_Touch_LCD_7_Board board;
static ESP32S3_Touch_LCD_7 display(board, 0, 0);

// Size of the LVGL PARTIAL draw band (internal SRAM), used only for 90/270
// rotation.  Bigger = longer contiguous forward PSRAM write-runs during the
// rotate-transpose, at the cost of internal RAM.  LVGL derives the band height
// from this and the logical (portrait) width.  Reduce if you run out of
// internal RAM at boot.
#ifndef LV_SETUP_ROT_BAND_BYTES
#define LV_SETUP_ROT_BAND_BYTES (80 * 1024)
#endif

class lv_setup_class {
public:
    // rotation: 0 / 180 are handled in the driver (DIRECT render, forward PSRAM
    //           read -- see display.begin()).  90 / 270 are handled here: LVGL
    //           renders PARTIAL in portrait and our flush rotates each region
    //           into the landscape framebuffers, reconciling the two buffers
    //           with a deferred dirty-rectangle scheme (see the rotate header).
    //           The index aliases 1/2/3 are accepted for 90/180/270.
    void begin(uint16_t rotation = 0) {
        rotation = cglcd7_normalize_rotation(rotation);  // 1/2/3 -> 90/180/270
        _rotation = rotation;
        bool quarter = (rotation == 90 || rotation == 270);

        // For 90/270 the driver stays at 0 (it reads PSRAM forward); rotation is
        // done above it.  For 0/180 the driver does the flip itself.
        uint16_t driverRotation = quarter ? 0 : rotation;
        if (!display.begin(16000000, driverRotation)) {
            Serial0.println("Display init failed!");
            while (1) delay(100);
        }
        display.allocateSecondFramebuffer();
        Serial0.println("Display initialized");
        display.setBacklight(true);

        // Touch is left in pass-through (raw physical/landscape coordinates);
        // _touchpad_read maps them to the active rotation below.
        _touch.begin();

        lv_init();

        if (!quarter) {
            // ---- 0 / 180: DIRECT render straight into the two framebuffers ----
            lv_display_t *disp = lv_display_create(800, 480);
            lv_display_set_buffers(disp, display.getFramebuffer(), display.getFramebuffer2(),
                display.height() * display.width() * sizeof(lv_color_t),
                LV_DISPLAY_RENDER_MODE_DIRECT);
            lv_display_set_flush_cb(disp, _disp_flush_direct);
            lv_display_set_user_data(disp, this);
        } else {
            // ---- 90 / 270: PARTIAL render in portrait + rotate/reconcile ----
            _renderer = new LvglRotatedRenderer(display, rotation);

            // Create the display at the LOGICAL portrait size; our flush rotates
            // each partial area into the physical landscape framebuffers.
            lv_display_t *disp = lv_display_create(480, 800);

            // One PARTIAL draw band in internal SRAM (fast strided reads during
            // the transpose).  Single buffer is fine: the flush is synchronous
            // CPU work (no DMA to overlap).
            void *band = heap_caps_malloc(LV_SETUP_ROT_BAND_BYTES,
                                          MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
            if (!band) {
                Serial0.println("Failed to allocate rotation draw band (internal SRAM)");
                while (1) delay(100);
            }
            lv_display_set_buffers(disp, band, nullptr, LV_SETUP_ROT_BAND_BYTES,
                                   LV_DISPLAY_RENDER_MODE_PARTIAL);
            lv_display_set_flush_cb(disp, _disp_flush_rotated);
            lv_display_set_user_data(disp, this);
        }

        static lv_indev_t *indev = lv_indev_create();
        lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
        lv_indev_set_read_cb(indev, _touchpad_read);
        lv_indev_set_user_data(indev, this);

        lv_tick_set_cb(_tick_get);
    }

private:
    ESP32S3_Touch_LCD_7_Touch _touch{board};
    int _rotation = 0;
    LvglRotatedRenderer* _renderer = nullptr;

    static uint32_t _tick_get(void) { return millis(); }

    // 0/180 DIRECT flush: LVGL renders into a full framebuffer; on the last area
    // of the frame, reveal that buffer at vsync.
    static void _disp_flush_direct(lv_display_t *disp, const lv_area_t *area, uint8_t *pixelmap) {
        if (lv_display_flush_is_last(disp)) {
            display.setActiveFramebuffer(
                ((void *)pixelmap == (void *)(display.getFramebuffer())) ? 1 : 2, true);
        }
        lv_disp_flush_ready(disp);
    }

    // 90/270 PARTIAL flush: rotate the area into the buffer we're building;
    // reconcile + swap on the last area of the frame.
    static void _disp_flush_rotated(lv_display_t *disp, const lv_area_t *area, uint8_t *pixelmap) {
        auto *self = (lv_setup_class *)lv_display_get_user_data(disp);
        self->_renderer->flushRegion(area, pixelmap);
        if (lv_display_flush_is_last(disp)) self->_renderer->frameDone();
        lv_disp_flush_ready(disp);
    }

    static void _touchpad_read(lv_indev_t *indev, lv_indev_data_t *data) {
        auto *self = (lv_setup_class *)lv_indev_get_user_data(indev);
        static uint32_t lastReadTime;
        static bool lastWasTouched;
        static int32_t lastx, lasty;
        // Rate limiting to once every 20ms improves drag tracking
        if ((millis() - lastReadTime) > 20) {
            self->_touch.read();
            lastWasTouched = self->_touch.isTouched;
            if (lastWasTouched) {
                // The GT911 reports raw physical (landscape) coordinates; map them
                // to the active rotation so they line up with the LVGL UI.  If
                // touch comes out mirrored on your board, swap the 90/270 cases.
                int rx = self->_touch.points[0].x;
                int ry = self->_touch.points[0].y;
                int W = display.width();    // 800
                int H = display.height();   // 480
                switch (self->_rotation) {
                    case 90:  lastx = (H - 1) - ry; lasty = rx;             break;
                    case 180: lastx = (W - 1) - rx; lasty = (H - 1) - ry;   break;
                    case 270: lastx = ry;           lasty = (W - 1) - rx;   break;
                    default:  lastx = rx;           lasty = ry;             break;  // 0
                }
            }
            lastReadTime = millis();
        }
        if (lastWasTouched) {
            data->state = LV_INDEV_STATE_PR;
            data->point.x = lastx;
            data->point.y = lasty;
        } else {
            data->state = LV_INDEV_STATE_REL;
        }
    }
};

static lv_setup_class lv_setup;
