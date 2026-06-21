#pragma once

// LVGL display and touch driver for the Waveshare ESP32-S3-Touch-LCD-7

#include <ESP32S3_Touch_LCD_7_Board.h>            // shared I2C bus + expander
#include <chipguy_ESP32S3_Touch_LCD_7_display.h>
#include <chipguy_ESP32S3_Touch_LCD_7_touch.h>
#include <Wire.h>
#include <lvgl.h>

// The board owns the shared I2C bus and the CH422G expander (display reset/
// backlight and touch reset).  The display and touch drivers reach their
// hardware through it.
static ESP32S3_Touch_LCD_7_Board board;
static ESP32S3_Touch_LCD_7 display(board, 0, 0);

class lv_setup_class {
public:
    void begin() {
        if (!display.begin()) {
            Serial0.println("Display init failed!");
            while(1) delay(100);
        }
        display.allocateSecondFramebuffer();
        Serial0.println("Display initialized");
        display.setBacklight(true);

        _touch.begin();

        lv_init();

        static lv_display_t *disp = lv_display_create(800, 480);
        lv_display_set_buffers(disp, display.getFramebuffer(), display.getFramebuffer2(),
            display.height() * display.width() * sizeof(lv_color_t), LV_DISPLAY_RENDER_MODE_DIRECT);
        lv_display_set_flush_cb(disp, _disp_flush);
        lv_display_set_user_data(disp, this);

        static lv_indev_t *indev = lv_indev_create();
        lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
        lv_indev_set_read_cb(indev, _touchpad_read);
        lv_indev_set_user_data(indev, this);

        lv_tick_set_cb(_tick_get);
    }

private:
    ESP32S3_Touch_LCD_7_Touch _touch{board};

    static uint32_t _tick_get(void) { return millis(); }

    static void _disp_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *pixelmap) {
        if (lv_display_flush_is_last(disp)) {
            display.setActiveFramebuffer(
                ((void *)pixelmap == (void *)(display.getFramebuffer())) ? 1 : 2, true);
        }
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
                // The GT911 reports coordinates already aligned with the panel
                // (the touch driver passes them through), so use them directly.
                // If touch comes out mirrored/rotated on your board, adjust here.
                lastx = self->_touch.points[0].x;
                lasty = self->_touch.points[0].y;
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
