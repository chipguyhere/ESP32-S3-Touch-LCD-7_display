/*
 * chipguy_ESP32S3_Touch_LCD_7 - Display driver for the Waveshare
 * ESP32-S3-Touch-LCD-7 board (ST7262 800x480 RGB LCD).
 *
 * The panel is a plain RGB-interface TFT with no command controller, so unlike
 * the ST7701-based 2.8C board there is no SPI bring-up sequence: the ESP32-S3
 * RGB LCD peripheral drives the panel directly.  The panel reset and the LCD
 * backlight enable are reached through the board's CH422G I2C expander.
 *
 * Copyright (c) 2025 chipguyhere
 * MIT License
 */

#include <Arduino.h>
#include "chipguy_ESP32S3_Touch_LCD_7_display.h"
#include "esp_heap_caps.h"
#include "esp_cpu.h"
#include "freertos/portmacro.h"

#include "esp_lcd_panel_rgb.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_private/gdma.h"
#include "hal/lcd_hal.h"
#include "hal/lcd_ll.h"
#include "soc/gdma_struct.h"
#include "hal/gdma_ll.h"

// Cache writeback function from ROM
#include "esp32s3/rom/cache.h"
extern "C" int Cache_WriteBack_Addr(uint32_t addr, uint32_t size);


// Helper macros for sprite coordinate/dimension packing
#define SPRITE_YX(y, x)     (((uint32_t)(uint16_t)(y) << 16) | (uint16_t)(x))
#define SPRITE_HW(h, w)     (((uint32_t)(uint16_t)(h) << 16) | (uint16_t)(w))
#define SPRITE_GET_Y(yx)    ((int16_t)((yx) >> 16))
#define SPRITE_GET_X(yx)    ((int16_t)((yx) & 0xFFFF))
#define SPRITE_GET_H(hw)    ((uint16_t)((hw) >> 16))
#define SPRITE_GET_W(hw)    ((uint16_t)((hw) & 0xFFFF))

// Pin definitions for the Waveshare ESP32-S3-Touch-LCD-7 board.
//
// This is a 16-bit parallel RGB panel (ST7262).  The panel reset and backlight
// are NOT on ESP32 GPIOs -- they hang off the CH422G I2C expander (LCD reset =
// IO3, backlight = IO2), driven via ESP32S3_Touch_LCD_7_Board.  There is no
// chip-select or SPI: the RGB sync/data lines below are the entire interface.
#define PIN_LCD_DE       5
#define PIN_LCD_VSYNC    3
#define PIN_LCD_HSYNC    46
#define PIN_LCD_PCLK     7
// RGB565 data lines.  data_gpio_nums[0..15] = B0..B4, G0..G5, R0..R4.
#define PIN_LCD_B0       14
#define PIN_LCD_B1       38
#define PIN_LCD_B2       18
#define PIN_LCD_B3       17
#define PIN_LCD_B4       10
#define PIN_LCD_G0       39
#define PIN_LCD_G1       0
#define PIN_LCD_G2       45
#define PIN_LCD_G3       48
#define PIN_LCD_G4       47
#define PIN_LCD_G5       21
#define PIN_LCD_R0       1
#define PIN_LCD_R1       2
#define PIN_LCD_R2       42
#define PIN_LCD_R3       41
#define PIN_LCD_R4       40


// Cache writeback helper - writes back a memory region with interrupt protection
// The ISR reads through the same CPU cache, so it sees dirty cache lines automatically.
// This writeback may be unnecessary - disable by uncommenting the return statement.
static portMUX_TYPE s_cache_mux = portMUX_INITIALIZER_UNLOCKED;
static inline void cacheWriteBack(const void* addr, size_t size) {
    return;  // DISABLED FOR TESTING - ISR should see dirty cache lines directly
    portENTER_CRITICAL(&s_cache_mux);
    Cache_WriteBack_Addr((uint32_t)addr, size);
    portEXIT_CRITICAL(&s_cache_mux);
}

// ISR timing instrumentation (cycle counts at 240MHz)
volatile uint64_t g_isrCyclesIn = 0;      // Total cycles spent inside ISR
volatile uint64_t g_isrCyclesOut = 0;     // Total cycles spent outside ISR
volatile uint32_t g_isrLastExit = 0;      // Cycle count when ISR last exited (32-bit is fine for deltas)
volatile uint32_t g_isrCallCount = 0;     // Number of ISR calls

// ---------------------------------------------------------------------------
// Bounce buffer desync detection and workaround
// ---------------------------------------------------------------------------
// PROBLEM: The ESP-IDF RGB LCD driver uses two bounce buffers (A and B) in a
// ping-pong arrangement. Normally the callback fills A while DMA sends B, then
// fills B while DMA sends A. Under certain conditions (e.g. flash/LittleFS
// writes introducing delays), the driver's phase can shift so the callback is
// asked to fill the SAME buffer the DMA is currently sending. This manifests
// as the display being vertically offset by the bounce buffer height (20 lines)
// with streaking artifacts where the CPU write races the DMA read.
//
// DETECTION: During begin(), we extract the GDMA channel handle from the
// panel's internal struct and cache a pointer to the GDMA TX out.dscr hardware
// register. In the ISR callback, we read this register to get the address of
// the DMA descriptor currently being processed, then dereference it to get the
// buffer address the DMA is actively reading from. If that address falls within
// the bounce buffer we've been asked to fill, we have a confirmed phase desync.
//
// WORKAROUND: When desync is detected, we redirect all writes (framebuffer
// copy, sprite overlay) to the OTHER bounce buffer, which is idle. This avoids
// corrupting the in-flight DMA transfer and places the data where the DMA will
// read it on its next pass.
//
// ESP-IDF DEPENDENCY: This workaround depends on:
//   1. The layout of esp_rgb_panel_t (esp_lcd_panel_rgb.c) to extract dma_chan.
//      See esp_rgb_panel_partial_t below -- must match ESP-IDF's struct layout.
//   2. The GDMA out.dscr register containing the current descriptor address.
//   3. The DMA descriptor layout: word0=flags, word1=buffer ptr, word2=next.
// If a future ESP-IDF version fixes the desync bug or changes any of these
// internals, this workaround can be removed by:
//   - Deleting this block of globals and the esp_rgb_panel_partial_t struct
//   - Deleting the desync detection + redirect block in bounceBufferFillCallback
//   - Deleting the GDMA setup block in begin()
//   - Removing the #include "soc/gdma_struct.h" and #include "hal/gdma_ll.h"
//   - Removing the g_bbDesyncCount / g_bbDiag_* externs from the header
// ---------------------------------------------------------------------------
volatile uint32_t g_bbDesyncCount = 0;    // Number of desync events detected
static volatile uint32_t* s_gdma_out_dscr_reg = nullptr;  // Pointer to GDMA TX out.dscr register
static void* s_bb_addrs[2] = {nullptr, nullptr};           // Learned bounce buffer addresses
static size_t s_bb_size = 0;                                // Bounce buffer size in bytes

// Diagnostic: last values seen by ISR (readable from main loop for debugging)
volatile uint32_t g_bbDiag_dscr = 0;        // Last descriptor address read from GDMA
volatile uint32_t g_bbDiag_bufAddr = 0;     // Buffer address extracted from that descriptor
volatile uint32_t g_bbDiag_bounceBuf = 0;   // bounce_buf pointer we were asked to fill

// ---------------------------------------------------------------------------
// Flash write overlay message
// ---------------------------------------------------------------------------
// When s_flashMsgBitmap is non-null, the ISR displays a black screen with a
// centered message, reading only from SRAM (no PSRAM or flash access).
// The bitmap is 1bpp (1 bit per pixel, MSB first, rows padded to byte boundary),
// pre-rendered by beginFlashMessage() and freed by endFlashMessage().
// The ISR expands 1bpp to RGB565 (0=black, 1=white) row by row.
// Double-buffered: main code writes to pending, ISR copies to active at frame start.
// This ensures the transition happens at a frame boundary (no tearing).
static volatile uint8_t* s_flashMsgActive = nullptr;   // active bitmap (ISR reads)
static volatile uint8_t* s_flashMsgPending = nullptr;  // pending bitmap (main code writes)
static int s_flashMsgW = 0;          // width of message in pixels
static int s_flashMsgH = 0;          // height of message in pixels
static int s_flashMsgStride = 0;     // bytes per row (ceil(width/8))
static int s_flashMsgX = 0;          // x offset to center on screen
static int s_flashMsgY = 0;          // y offset to center on screen
static bool s_flashMsgTile = false;  // if true, tile message every bounce buffer band
// Sentinel value used when beginFlashMessage(-1) blanks screen without text.
// Must be non-null so the ISR enters the flash overlay path, but not heap-allocated.
static uint8_t s_blankSentinel = 0;

// Partial mirror of esp_rgb_panel_t from esp_lcd_panel_rgb.c (ESP-IDF v5.5)
// Only fields up through dma_chan are needed so we can extract the GDMA channel
// handle at init time. We use byte arrays and void* for types whose definitions
// are not available in public headers (esp_lcd_panel_t, intr_handle_t, etc.),
// keeping the sizes identical to the real types (all pointer-sized on ESP32-S3).
// WARNING: This struct MUST be kept in sync with the real esp_rgb_panel_t.
// If the ESP-IDF struct layout changes, the dma_chan offset will be wrong and
// the gdma_get_group_channel_id() call in begin() will fail (logged to Serial).
struct esp_rgb_panel_partial_t {
    uint8_t base[10 * sizeof(void*)]; // esp_lcd_panel_t: 9 fn ptrs + user_data
    int panel_id;
    void* hal_dev;                  // lcd_hal_context_t contains one pointer
    size_t data_width;
    size_t fb_bits_per_pixel;
    size_t num_fbs;
    size_t output_bits_per_pixel;
    size_t dma_burst_size;
    int disp_gpio_num;
    void* intr;                     // intr_handle_t (opaque pointer)
    void* pm_lock;                  // esp_pm_lock_handle_t (opaque pointer)
    size_t num_dma_nodes;
    gdma_channel_handle_t dma_chan;  // <-- needed for desync detection
    void* dma_fb_links[3];          // gdma_link_list_handle_t dma_fb_links[RGB_LCD_PANEL_MAX_FB_NUM]
    void* dma_bb_link;              // gdma_link_list_handle_t dma_bb_link
    void* dma_restart_link;         // gdma_link_list_handle_t dma_restart_link (ESP32-S3 only)
    uint8_t* fbs[3];               // uint8_t *fbs[RGB_LCD_PANEL_MAX_FB_NUM]
    uint8_t* bounce_buffer[2];     // <-- needed to prefill "MUST ENABLE PSRAM" message
    size_t fb_size;
    size_t bb_size;
};

// ---------------------------------------------------------------------------
// "MUST ENABLE PSRAM" error message for when PSRAM is unavailable
// ---------------------------------------------------------------------------
// Minimal 5x7 font covering only the characters needed for the message.
// Each character is 7 bytes (one per row), with pixel data in bits 7..3 (MSB=left).
// Rendered at 2x scale (10x14 per char) centered on the 800px-wide screen.
static const uint8_t s_charA[] = {0x70,0x88,0x88,0xF8,0x88,0x88,0x88};
static const uint8_t s_charB[] = {0xF0,0x88,0x88,0xF0,0x88,0x88,0xF0};
static const uint8_t s_charE[] = {0xF8,0x80,0x80,0xF0,0x80,0x80,0xF8};
static const uint8_t s_charF[] = {0xF8,0x80,0x80,0xF0,0x80,0x80,0x80};
static const uint8_t s_charH[] = {0x88,0x88,0x88,0xF8,0x88,0x88,0x88};
static const uint8_t s_charI[] = {0x70,0x20,0x20,0x20,0x20,0x20,0x70};
static const uint8_t s_charL[] = {0x80,0x80,0x80,0x80,0x80,0x80,0xF8};
static const uint8_t s_charM[] = {0x88,0xD8,0xA8,0xA8,0x88,0x88,0x88};
static const uint8_t s_charN[] = {0x88,0xC8,0xA8,0x98,0x88,0x88,0x88};
static const uint8_t s_charO[] = {0x70,0x88,0x88,0x88,0x88,0x88,0x70};
static const uint8_t s_charP[] = {0xF0,0x88,0x88,0xF0,0x80,0x80,0x80};
static const uint8_t s_charR[] = {0xF0,0x88,0x88,0xF0,0xA0,0x90,0x88};
static const uint8_t s_charS[] = {0x70,0x88,0x80,0x70,0x08,0x88,0x70};
static const uint8_t s_charT[] = {0xF8,0x20,0x20,0x20,0x20,0x20,0x20};
static const uint8_t s_charU[] = {0x88,0x88,0x88,0x88,0x88,0x88,0x70};
static const uint8_t s_charW[] = {0x88,0x88,0x88,0xA8,0xA8,0xD8,0x88};
static const uint8_t s_charY[] = {0x88,0x88,0x50,0x20,0x20,0x20,0x20};
static const uint8_t s_charSP[]= {0x00,0x00,0x00,0x00,0x00,0x00,0x00};

// "MUST ENABLE PSRAM" = 17 characters (including spaces)
static const uint8_t* const s_noPsramMsg[] = {
    s_charM,s_charU,s_charS,s_charT,s_charSP,
    s_charE,s_charN,s_charA,s_charB,s_charL,s_charE,s_charSP,
    s_charP,s_charS,s_charR,s_charA,s_charM
};
static const int s_noPsramMsgLen = sizeof(s_noPsramMsg) / sizeof(s_noPsramMsg[0]);
// At 2x scale: each char is 10px wide + 2px gap = 12px per char.
// 17 chars * 12 - 2 (no trailing gap) = 202px wide, 14px tall.
// Centered on 800px screen: x_offset = (800 - 202) / 2 = 299
// The two bounce buffers span 40 lines, so the message tiles every 40 lines.
static const int s_noPsramScale = 2;
static const int s_noPsramCharW = 5 * s_noPsramScale;       // 10px
static const int s_noPsramCharH = 7 * s_noPsramScale;       // 14px
static const int s_noPsramGap = s_noPsramScale;             // 2px gap between chars
static const int s_noPsramTotalW = s_noPsramMsgLen * (s_noPsramCharW + s_noPsramGap) - s_noPsramGap;
static const int s_noPsramXOffset = (800 - s_noPsramTotalW) / 2;
// Vertical: center the 14px-tall text in a 40px band, repeating every 40 lines
static const int s_noPsramBandH = 40;
static const int s_noPsramYOffset = (s_noPsramBandH - s_noPsramCharH) / 2;

// "FLASH MEMORY WRITE" = 18 characters (including spaces)
// Used by beginFlashMessage() / endFlashMessage() to show an overlay during flash ops.
static const uint8_t* const s_flashWriteMsg[] = {
    s_charF,s_charL,s_charA,s_charS,s_charH,s_charSP,
    s_charM,s_charE,s_charM,s_charO,s_charR,s_charY,s_charSP,
    s_charW,s_charR,s_charI,s_charT,s_charE
};
static const int s_flashWriteMsgLen = sizeof(s_flashWriteMsg) / sizeof(s_flashWriteMsg[0]);

// Pre-render a font message into a 1bpp bitmap allocated in SRAM.
// Returns the buffer (caller must free), and fills in width, height, stride.
// 1bpp format: MSB first, rows padded to byte boundary.
// scale: pixel scale factor (2 = 10x14 per character).
// NOT IRAM_ATTR -- runs from flash at call time, not from ISR.
static uint8_t* renderMessageTo1bpp(const uint8_t* const* msg, int msgLen,
                                     int scale, int* outW, int* outH, int* outStride) {
    int charW = 5 * scale;
    int charH = 7 * scale;
    int gap = scale;
    int totalW = msgLen * (charW + gap) - gap;
    int totalH = charH;
    int stride = (totalW + 7) / 8;  // bytes per row, rounded up
    *outW = totalW;
    *outH = totalH;
    *outStride = stride;
    uint8_t* buf = (uint8_t*)heap_caps_malloc(stride * totalH,
                                               MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!buf) return nullptr;
    memset(buf, 0, stride * totalH);
    for (int y = 0; y < totalH; y++) {
        int src_row = y / scale;
        for (int x = 0; x < totalW; x++) {
            int char_cell = x / (charW + gap);
            int in_cell_x = x % (charW + gap);
            if (char_cell < msgLen && in_cell_x < charW) {
                int src_col = in_cell_x / scale;
                uint8_t row_bits = msg[char_cell][src_row];
                if (row_bits & (0x80 >> src_col)) {
                    buf[y * stride + x / 8] |= (0x80 >> (x % 8));
                }
            }
        }
    }
    return buf;
}

// Render "MUST ENABLE PSRAM" message into a bounce buffer region.
// Called once per bounce buffer during init (not from ISR) to prefill the
// buffers with the error message. The DMA then continuously displays them.
// NOT IRAM_ATTR -- this runs from flash at init time only, saving IRAM.
static void renderNoPsramMessage(uint16_t* dst, int pos_px, int pixel_count, int screen_width) {
    for (int i = 0; i < pixel_count; i++) {
        int scr_x = (pos_px + i) % screen_width;
        int scr_y = (pos_px + i) / screen_width;
        int band_y = scr_y % s_noPsramBandH;
        int font_y = band_y - s_noPsramYOffset;
        uint16_t color = 0x0000; // black background
        if (font_y >= 0 && font_y < s_noPsramCharH) {
            int src_row = font_y / s_noPsramScale;
            int msg_x = scr_x - s_noPsramXOffset;
            if (msg_x >= 0 && msg_x < s_noPsramTotalW) {
                int char_cell = msg_x / (s_noPsramCharW + s_noPsramGap);
                int in_cell_x = msg_x % (s_noPsramCharW + s_noPsramGap);
                if (char_cell < s_noPsramMsgLen && in_cell_x < s_noPsramCharW) {
                    int src_col = in_cell_x / s_noPsramScale;
                    uint8_t row_bits = s_noPsramMsg[char_cell][src_row];
                    if (row_bits & (0x80 >> src_col)) {
                        color = 0xFFFF; // white text
                    }
                }
            }
        }
        dst[i] = color;
    }
}

// Bounce buffer callback - copies pixels from PSRAM framebuffer to bounce buffer
// Uses linear ring buffer addressing: each screen row is contiguous, only wraps at buffer end
bool IRAM_ATTR bounceBufferFillCallback(esp_lcd_panel_handle_t panel, void *bounce_buf,
                                                int pos_px, int len_bytes, void *user_ctx) {
    uint32_t entry_time = esp_cpu_get_cycle_count();

    // Accumulate time spent outside ISR (since last exit)
    if (g_isrCallCount > 0) {
        g_isrCyclesOut += (entry_time - g_isrLastExit);
    }
    g_isrCallCount=g_isrCallCount+1;

    ESP32S3_Touch_LCD_7* display = (ESP32S3_Touch_LCD_7*)user_ctx;

    // --- Begin bounce buffer desync detection and workaround ---
    // (See block comment above for full explanation; removable if ESP-IDF fixes this)

    // Learn both bounce buffer addresses from the first two distinct callback invocations
    if (s_bb_addrs[0] == nullptr) {
        s_bb_addrs[0] = bounce_buf;
    } else if (s_bb_addrs[1] == nullptr && bounce_buf != s_bb_addrs[0]) {
        s_bb_addrs[1] = bounce_buf;
    }

    // Read the GDMA out.dscr register to find the descriptor currently being DMA'd,
    // then extract the buffer address from that descriptor (word 1 of the DMA descriptor).
    // If the DMA's active buffer is the same one we've been asked to fill, the driver's
    // ping-pong phase is inverted -- redirect our writes to the other (idle) buffer.
    bool desync = false;
    if (s_gdma_out_dscr_reg != nullptr && s_bb_size > 0) {
        uint32_t dscr_addr = *s_gdma_out_dscr_reg;
        g_bbDiag_dscr = dscr_addr;
        g_bbDiag_bounceBuf = (uint32_t)bounce_buf;
        if (dscr_addr != 0) {
            uint32_t* desc = (uint32_t*)dscr_addr;
            uint32_t dma_buf_addr = desc[1];  // DMA descriptor word 1 = buffer pointer
            g_bbDiag_bufAddr = dma_buf_addr;
            uint32_t bb_start = (uint32_t)bounce_buf;
            uint32_t bb_end = bb_start + s_bb_size;
            if (dma_buf_addr >= bb_start && dma_buf_addr < bb_end) {
                desync = true;
                g_bbDesyncCount = g_bbDesyncCount + 1;
                // Redirect writes to the other bounce buffer since this one is in-flight
                if (s_bb_addrs[0] != nullptr && s_bb_addrs[1] != nullptr) {
                    bounce_buf = (bounce_buf == s_bb_addrs[0]) ? s_bb_addrs[1] : s_bb_addrs[0];
                }
            }
        }
    }
    // --- End bounce buffer desync detection and workaround ---

    // At frame start (pos_px == 0), copy pending values to active
    // This provides tear-free scrolling/swapping by only changing at frame boundaries
    if (pos_px == 0) {
        display->_scroll_offset = display->_pending_scroll_offset;
        display->_active_fb = display->_pending_fb;
        s_flashMsgActive = s_flashMsgPending;

        // Copy pending sprites to active sprites for this frame
        for (int i = 0; i < SPRITE_MAX_COUNT; i++) {
            display->_active_sprites[i] = display->_pending_sprites[i];
        }
    }

    // Select the active framebuffer (1 = primary, 2 = secondary)
    uint16_t* framebuffer = (display->_active_fb == 2 && display->_framebuffer2 != nullptr)
                            ? display->_framebuffer2
                            : display->_framebuffer;

    const int screen_width = ESP32S3_Touch_LCD_7::_width;

    // If no framebuffer (PSRAM not enabled), the bounce buffers were prefilled
    // with the "MUST ENABLE PSRAM" message during begin(). Nothing to do here --
    // the DMA just keeps cycling the prefilled buffers.
    if (framebuffer == nullptr) {
        return false;
    }

    // Flash write overlay: if active, fill bounce buffer with black background
    // and expand the 1bpp SRAM bitmap for message rows that overlap this chunk.
    // Row-by-row copy from SRAM only -- no PSRAM or flash access.
    const volatile uint8_t* flashBmp = s_flashMsgActive;
    if (flashBmp != nullptr) {
        uint16_t* dst = (uint16_t*)bounce_buf;
        int pixel_count = len_bytes / sizeof(uint16_t);
        int start_y = pos_px / screen_width;
        int num_lines = pixel_count / screen_width;

        // Fill entire bounce buffer black (32-bit fill for speed)
        uint32_t* dst32 = (uint32_t*)dst;
        for (int i = 0; i < pixel_count / 2; i++) {
            dst32[i] = 0;
        }

        // Expand 1bpp message rows that fall within this chunk.
        // If tiling, the message repeats every num_lines lines (bounce buffer height),
        // so every chunk shows it at the same offset -- even if the DMA replays a
        // single chunk during a flash stall, the message is always visible.
        int msgX = s_flashMsgX;
        int msgY = s_flashMsgY;
        int msgW = s_flashMsgW;
        int msgH = s_flashMsgH;
        int stride = s_flashMsgStride;
        bool tile = s_flashMsgTile;
        for (int line = 0; line < num_lines; line++) {
            int scr_y = start_y + line;
            int rel_y;
            if (tile) {
                // Tile: use position within the bounce buffer band, offset by msgY
                // which is the offset within a band (set by beginFlashMessage)
                int band_y = scr_y % num_lines;
                rel_y = band_y - msgY;
            } else {
                rel_y = scr_y - msgY;
            }
            if (rel_y < 0 || rel_y >= msgH) continue;
            const volatile uint8_t* src_row = flashBmp + rel_y * stride;
            uint16_t* dst_row = dst + line * screen_width + msgX;
            for (int x = 0; x < msgW; x++) {
                if (src_row[x / 8] & (0x80 >> (x % 8))) {
                    dst_row[x] = 0xFFFF;
                }
            }
        }
        return false;
    }

    const uint32_t scroll_offset = display->_scroll_offset;
    const uint32_t fb_size = display->_fb_size;
    const int fb_stride = display->_fb_stride;

    uint16_t* dst = (uint16_t*)bounce_buf;
    int pixel_count = len_bytes / sizeof(uint16_t);

    // Screen position from linear pos_px
    int screen_y = pos_px / screen_width;
    int screen_x = pos_px % screen_width;

    // Linear framebuffer position: scroll_offset + screen_y * fb_stride + screen_x
    // Each screen row is contiguous (800 pixels), with fb_stride spacing between rows
    while (pixel_count > 0) {
        // Compute linear position in framebuffer
        uint32_t fb_pos = (scroll_offset + screen_y * fb_stride + screen_x) % fb_size;
        uint16_t* src = framebuffer + fb_pos;

        // How many pixels until end of screen row or buffer wrap?
        int pixels_to_row_end = screen_width - screen_x;
        int pixels_to_wrap = fb_size - fb_pos;
        int segment = pixels_to_row_end;
        if (pixels_to_wrap < segment) segment = pixels_to_wrap;
        if (pixel_count < segment) segment = pixel_count;

        // Copy segment using 32-bit operations
        int remaining = segment;

        // Align dst if needed
        if (((uintptr_t)dst & 2) && remaining > 0) {
            *dst++ = *src++;
            remaining--;
        }

        // Align src if needed
        if (((uintptr_t)src & 2) && remaining > 0) {
            *dst++ = *src++;
            remaining--;
        }

        // Now both are 32-bit aligned - fast copy
        uint32_t* src32 = (uint32_t*)src;
        uint32_t* dst32 = (uint32_t*)dst;
        while (remaining >= 8) {
            dst32[0] = src32[0];
            dst32[1] = src32[1];
            dst32[2] = src32[2];
            dst32[3] = src32[3];
            src32 += 4;
            dst32 += 4;
            remaining -= 8;
        }
        while (remaining >= 2) {
            *dst32++ = *src32++;
            remaining -= 2;
        }
        dst = (uint16_t*)dst32;
        if (remaining) {
            *dst++ = *(uint16_t*)src32;
        }

        pixel_count -= segment;
        screen_x += segment;
        if (screen_x >= screen_width) {
            screen_x = 0;
            screen_y++;
        }
    }

    // Sprite overlay - draw sprites on top of framebuffer content
    // Sprites are in display coordinates (not affected by scrolling)
    const int chunk_start_px = pos_px;
    const int chunk_end_px = pos_px + (len_bytes / sizeof(uint16_t)) - 1;
    const int chunk_start_y = chunk_start_px / screen_width;
    const int chunk_end_y = chunk_end_px / screen_width;

    uint16_t* bounce_buf16 = (uint16_t*)bounce_buf;

    for (int spr = 0; spr < SPRITE_MAX_COUNT; spr++) {
        const Sprite& sprite = display->_active_sprites[spr];

        // Skip disabled sprites
        if (sprite.yx == SPRITE_DISABLED || sprite.data == nullptr) continue;

        // Extract sprite properties
        const int16_t spr_y = SPRITE_GET_Y(sprite.yx);
        const int16_t spr_x = SPRITE_GET_X(sprite.yx);
        const uint16_t spr_h = SPRITE_GET_H(sprite.hw);
        const uint16_t spr_w = SPRITE_GET_W(sprite.hw);
        const uint16_t trans = sprite.transparent;
        const uint16_t* spr_data = sprite.data;
        const bool detect_collision = (sprite.flags & SPRITE_FLAG_COLLISION_DETECT) != 0;

        // Collision detection state for this sprite in this chunk
        bool first_color_seen = false;
        uint16_t first_color = 0;
        bool collision_detected = false;

        // Calculate sprite's screen Y range
        const int16_t spr_y_end = spr_y + spr_h - 1;

        // Quick rejection: sprite doesn't overlap chunk's Y range
        if (spr_y > chunk_end_y || spr_y_end < chunk_start_y) continue;

        // Calculate sprite's screen X range (for horizontal bounds)
        const int16_t spr_x_end = spr_x + spr_w - 1;

        // Quick rejection: sprite is entirely off-screen horizontally
        if (spr_x >= screen_width || spr_x_end < 0) continue;

        // Process each row of the sprite that falls within this chunk
        for (int16_t row = 0; row < spr_h; row++) {
            int16_t scr_y = spr_y + row;

            // Skip rows outside chunk's Y range
            if (scr_y < chunk_start_y || scr_y > chunk_end_y) continue;

            // Skip rows off-screen
            if (scr_y < 0 || scr_y >= ESP32S3_Touch_LCD_7::_height) continue;

            // Calculate visible X range for this sprite row
            int16_t vis_x_start = (spr_x < 0) ? 0 : spr_x;
            int16_t vis_x_end = (spr_x_end >= screen_width) ? (screen_width - 1) : spr_x_end;

            // Skip if no visible pixels
            if (vis_x_start > vis_x_end) continue;

            // Calculate bounce buffer position for this screen row/col
            int row_start_px = scr_y * screen_width;  // Start of this screen row in linear coords

            // Only process columns that fall within this chunk
            int chunk_row_start = row_start_px;
            int chunk_row_end = row_start_px + screen_width - 1;

            // Clip to actual chunk bounds
            if (chunk_row_start < chunk_start_px) chunk_row_start = chunk_start_px;
            if (chunk_row_end > chunk_end_px) chunk_row_end = chunk_end_px;

            // For each pixel in the visible sprite row
            for (int16_t scr_x = vis_x_start; scr_x <= vis_x_end; scr_x++) {
                int linear_pos = scr_y * screen_width + scr_x;

                // Check if this pixel is in the current chunk
                if (linear_pos < chunk_start_px || linear_pos > chunk_end_px) continue;

                // Get sprite pixel
                int spr_px_x = scr_x - spr_x;  // X offset into sprite
                uint16_t spr_pixel = spr_data[row * spr_w + spr_px_x];

                // Skip transparent pixels
                if (spr_pixel == trans) continue;

                int buf_offset = linear_pos - chunk_start_px;

                // Collision detection: check if we're overwriting multiple colors
                if (detect_collision && !collision_detected) {
                    uint16_t existing_color = bounce_buf16[buf_offset];
                    if (!first_color_seen) {
                        first_color = existing_color;
                        first_color_seen = true;
                    } else if (existing_color != first_color) {
                        collision_detected = true;
                    }
                }

                // Write to bounce buffer
                bounce_buf16[buf_offset] = spr_pixel;
            }
        }

        // Set collision flag if detected (ISR only sets, never clears)
        if (collision_detected) {
            display->_sprite_collisions[spr] = 1;
        }
    }

    // Debug: uncomment to draw a green line on the last scanline of the chunk when
    // desync is detected. Drawn at the end of the buffer because the DMA reads forward
    // from the start, so pixels near the beginning may already have been sent.
    // if (desync) {
    //     int total_pixels = len_bytes / (int)sizeof(uint16_t);
    //     int last_line_start = total_pixels - screen_width;
    //     if (last_line_start < 0) last_line_start = 0;
    //     uint16_t* indicator = (uint16_t*)bounce_buf + last_line_start;
    //     uint16_t green = 0x07E0; // RGB565 green
    //     int count = total_pixels - last_line_start;
    //     for (int i = 0; i < count; i++) {
    //         indicator[i] = green;
    //     }
    // }

    // Accumulate time spent inside ISR and record exit time
    uint32_t exit_time = esp_cpu_get_cycle_count();
    g_isrCyclesIn += (exit_time - entry_time);
    g_isrLastExit = exit_time;

    return false;
}

ESP32S3_Touch_LCD_7::ESP32S3_Touch_LCD_7(ESP32S3_Touch_LCD_7_Board& board,
                                         uint16_t fb_pad_x, uint16_t fb_pad_y)
    : _board(board)
    , _fb_pad_x(fb_pad_x)
    , _fb_pad_y(fb_pad_y)
    , _fb_stride(_width + fb_pad_x)
    , _fb_rows(_height + fb_pad_y)
    , _fb_size((_width + fb_pad_x) * (_height + fb_pad_y))
{
    // Framebuffer includes padding for pre-drawing scroll content
    _framebuffer_size = _fb_size * sizeof(uint16_t);

    // Initialize all sprite slots as disabled
    for (int i = 0; i < SPRITE_MAX_COUNT; i++) {
        _pending_sprites[i].data = nullptr;
        _pending_sprites[i].yx = SPRITE_DISABLED;
        _pending_sprites[i].hw = 0;
        _pending_sprites[i].transparent = 0;
        _pending_sprites[i].flags = 0;
        _active_sprites[i].data = nullptr;
        _active_sprites[i].yx = SPRITE_DISABLED;
        _active_sprites[i].hw = 0;
        _active_sprites[i].transparent = 0;
        _active_sprites[i].flags = 0;
        _sprite_collisions[i] = 0;
    }
}

bool ESP32S3_Touch_LCD_7::allocateSecondFramebuffer() {
    // Already allocated
    if (_framebuffer2 != nullptr) {
        return false;
    }

    // Allocate second framebuffer with same size and alignment as primary
    _framebuffer2 = (uint16_t*)heap_caps_aligned_alloc(64, _framebuffer_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!_framebuffer2) {
        Serial.println("Failed to allocate second PSRAM framebuffer");
        return false;
    }

    // Clear to black
    memset(_framebuffer2, 0, _framebuffer_size);

    return true;
}

void ESP32S3_Touch_LCD_7::setDrawBuffer(uint8_t index) {
    if (index == 1) {
        _draw_target = _framebuffer;
        _draw_fb_index = 1;
    } else if (index == 2 && _framebuffer2 != nullptr) {
        _draw_target = _framebuffer2;
        _draw_fb_index = 2;
    }
}

void ESP32S3_Touch_LCD_7::setActiveFramebuffer(uint8_t index, bool wait) {
    // index: 0 = swap to other, 1 = primary, 2 = secondary
    if (index == 0) {
        // Swap: toggle between 1 and 2 (only if framebuffer2 exists)
        if (_framebuffer2 != nullptr) {
            _pending_fb = (_pending_fb == 1) ? 2 : 1;
        } else {
            // No second framebuffer, nothing to swap
            return;
        }
    } else if (index == 1) {
        _pending_fb = 1;
    } else if (index == 2 && _framebuffer2 != nullptr) {
        _pending_fb = 2;
    } else {
        // Invalid request - nothing to wait for
        return;
    }

    // If requested, block until the ISR has applied the pending value
    if (wait) {
        while (_active_fb != _pending_fb) {
            // Yield to other tasks while waiting for vsync
            vTaskDelay(1);
        }
    }
}

bool ESP32S3_Touch_LCD_7::begin(uint32_t pclk_hz) {
    // Bring up the board's shared I2C bus + CH422G expander first: on this board
    // the ST7262's reset and the LCD backlight live on the expander.
    // (Idempotent -- touch.begin() may also call it.)
    _board.begin();

    // Hardware-reset the panel via the expander (IO3).  There is no SPI command
    // controller on this RGB panel, so this is the entire panel bring-up.
    _board.lcdReset();

    // Allocate our own framebuffer in PSRAM (required for no_fb mode with callback)
    // Use 64-byte alignment for optimal cache line access
    // If PSRAM is not enabled or allocation fails, _framebuffer stays null and
    // the bounce buffer callback will display a "MUST ENABLE PSRAM" message instead.
    _framebuffer = (uint16_t*)heap_caps_aligned_alloc(64, _framebuffer_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!_framebuffer) {
        Serial.println("Failed to allocate PSRAM framebuffer - display will show error message");
    }

    // Set default draw target to primary framebuffer
    _draw_target = _framebuffer;
    _draw_fb_index = 1;

    // Configure RGB panel for the ST7262 800x480 panel (Waveshare's timing).
    esp_lcd_rgb_panel_config_t panel_config = {
        .clk_src = LCD_CLK_SRC_DEFAULT,
        .timings = {
            .pclk_hz = pclk_hz,
            .h_res = _width,
            .v_res = _height,
            .hsync_pulse_width = 4,
            .hsync_back_porch = 8,
            .hsync_front_porch = 8,
            .vsync_pulse_width = 4,
            .vsync_back_porch = 8,
            .vsync_front_porch = 8,
            .flags = {
                .hsync_idle_low = 0,
                .vsync_idle_low = 0,
                .de_idle_high = 0,
                .pclk_active_neg = 1,  // ST7262: data latched on falling edge
                .pclk_idle_high = 0,
            },
        },
        .data_width = 16,
        .bits_per_pixel = 16,
        .num_fbs = 0,
        .bounce_buffer_size_px = _bounce_buffer_size_px,  // 20 scanlines in SRAM
        .sram_trans_align = 8,
        .psram_trans_align = 64,
        .hsync_gpio_num = PIN_LCD_HSYNC,
        .vsync_gpio_num = PIN_LCD_VSYNC,
        .de_gpio_num = PIN_LCD_DE,
        .pclk_gpio_num = PIN_LCD_PCLK,
        .disp_gpio_num = -1,
        .data_gpio_nums = {0},
        .flags = {
            .disp_active_low = 0,
            .refresh_on_demand = false,
            .fb_in_psram = false,
            .double_fb = false,
            .no_fb = true,
            .bb_invalidate_cache = false,
        },
    };

    // Set data GPIO pins individually (RGB565: B=bits 0-4, G=bits 5-10, R=bits 11-15)
    panel_config.data_gpio_nums[0] = PIN_LCD_B0;
    panel_config.data_gpio_nums[1] = PIN_LCD_B1;
    panel_config.data_gpio_nums[2] = PIN_LCD_B2;
    panel_config.data_gpio_nums[3] = PIN_LCD_B3;
    panel_config.data_gpio_nums[4] = PIN_LCD_B4;
    panel_config.data_gpio_nums[5] = PIN_LCD_G0;
    panel_config.data_gpio_nums[6] = PIN_LCD_G1;
    panel_config.data_gpio_nums[7] = PIN_LCD_G2;
    panel_config.data_gpio_nums[8] = PIN_LCD_G3;
    panel_config.data_gpio_nums[9] = PIN_LCD_G4;
    panel_config.data_gpio_nums[10] = PIN_LCD_G5;
    panel_config.data_gpio_nums[11] = PIN_LCD_R0;
    panel_config.data_gpio_nums[12] = PIN_LCD_R1;
    panel_config.data_gpio_nums[13] = PIN_LCD_R2;
    panel_config.data_gpio_nums[14] = PIN_LCD_R3;
    panel_config.data_gpio_nums[15] = PIN_LCD_R4;

    esp_err_t ret = esp_lcd_new_rgb_panel(&panel_config, &_panel_handle);
    if (ret != ESP_OK) {
        Serial.printf("Failed to create RGB panel: %d\n", ret);
        return false;
    }

    // --- Begin desync detection setup (removable if ESP-IDF fixes bounce buffer phasing) ---
    // Extract the GDMA channel from the panel's internal struct so we can cache a pointer
    // to the hardware register (out.dscr) that reveals which buffer the DMA is reading from.
    {
        esp_rgb_panel_partial_t* rgb_panel = (esp_rgb_panel_partial_t*)_panel_handle;
        gdma_channel_handle_t dma_chan = rgb_panel->dma_chan;
        int group_id = -1, channel_id = -1;
        if (gdma_get_group_channel_id(dma_chan, &group_id, &channel_id) == ESP_OK) {
            gdma_dev_t* gdma_hw = GDMA_LL_GET_HW(group_id);
            if (gdma_hw != nullptr) {
                s_gdma_out_dscr_reg = &gdma_hw->channel[channel_id].out.dscr;
                Serial.printf("Desync detection: GDMA group=%d ch=%d, out.dscr @%p\n",
                              group_id, channel_id, s_gdma_out_dscr_reg);
            }
        } else {
            Serial.printf("Desync detection: gdma_get_group_channel_id failed (dma_chan=%p)\n", dma_chan);
        }
        // Bounce buffer size in bytes
        s_bb_size = panel_config.bounce_buffer_size_px * sizeof(uint16_t);
    }
    // --- End desync detection setup ---

    // Register bounce buffer callback BEFORE init (when refresh starts)
    // With no_fb=true, the callback will be invoked to fill bounce buffers
    // Pass 'this' so callback can access framebuffer and scroll offsets
    esp_lcd_rgb_panel_event_callbacks_t cbs = {};
    cbs.on_bounce_empty = bounceBufferFillCallback;
    ret = esp_lcd_rgb_panel_register_event_callbacks(_panel_handle, &cbs, this);
    if (ret != ESP_OK) {
        Serial.printf("Failed to register bounce callback: %d\n", ret);
        return false;
    }

    ret = esp_lcd_panel_reset(_panel_handle);
    if (ret != ESP_OK) {
        Serial.printf("Failed to reset panel: %d\n", ret);
        return false;
    }

    ret = esp_lcd_panel_init(_panel_handle);
    if (ret != ESP_OK) {
        Serial.printf("Failed to init panel: %d\n", ret);
        return false;
    }

    // If PSRAM allocation failed, prefill both bounce buffers with the
    // "MUST ENABLE PSRAM" error message. The DMA will cycle these forever
    // since the ISR does nothing when there's no framebuffer.
    if (!_framebuffer) {
        esp_rgb_panel_partial_t* rgb_panel = (esp_rgb_panel_partial_t*)_panel_handle;
        int bb_pixels = panel_config.bounce_buffer_size_px;
        // Each bounce buffer covers a different vertical slice of the screen.
        // Buffer 0: scanlines 0..19, buffer 1: scanlines 20..39, then they repeat.
        for (int b = 0; b < 2; b++) {
            if (rgb_panel->bounce_buffer[b]) {
                renderNoPsramMessage((uint16_t*)rgb_panel->bounce_buffer[b],
                                     b * bb_pixels, bb_pixels, _width);
            }
        }
    }

    // Clear screen to black
    fillScreen(0x0000);

    // Reveal the display: turn the backlight on (via the CH422G expander).
    setBacklight(100);

    return true;
}

void ESP32S3_Touch_LCD_7::setBacklight(int percentage) {
    // On this board the LCD backlight is a CH422G expander pin (no PWM), so
    // brightness is on/off only: any positive percentage turns it on.
    _board.lcdBacklight(percentage > 0);
}

void ESP32S3_Touch_LCD_7::setBacklight(bool on) {
    _board.lcdBacklight(on);
}

void ESP32S3_Touch_LCD_7::pushImage(int16_t x, int16_t y, int16_t w, int16_t h, const uint16_t* pixels) {
    if (_draw_target == nullptr) return;
    // Bounds checking against padded framebuffer size (allows pre-drawing scroll content)
    if (x >= _fb_stride || y >= _fb_rows || x + w <= 0 || y + h <= 0) {
        return;
    }

    // Clip to padded framebuffer bounds
    int16_t x1 = max((int16_t)0, x);
    int16_t y1 = max((int16_t)0, y);
    int16_t x2 = min((int16_t)(_fb_stride - 1), (int16_t)(x + w - 1));
    int16_t y2 = min((int16_t)(_fb_rows - 1), (int16_t)(y + h - 1));

    int16_t src_offset_x = x1 - x;
    int16_t src_offset_y = y1 - y;
    int16_t clipped_w = x2 - x1 + 1;
    int16_t clipped_h = y2 - y1 + 1;

    // Copy pixels row by row using linear addressing
    for (int16_t row = 0; row < clipped_h; row++) {
        const uint16_t* src = pixels + ((src_offset_y + row) * w) + src_offset_x;

        // Linear position in ring buffer
        uint32_t fb_pos = (_pending_scroll_offset + (y1 + row) * _fb_stride + x1) % _fb_size;

        // Each row is contiguous - only need to handle wrap at buffer end
        int16_t pixels_remaining = clipped_w;
        int16_t src_offset = 0;

        while (pixels_remaining > 0) {
            int pixels_to_wrap = _fb_size - fb_pos;
            int segment = (pixels_remaining < pixels_to_wrap) ? pixels_remaining : pixels_to_wrap;

            memcpy(_draw_target + fb_pos, src + src_offset, segment * sizeof(uint16_t));
            cacheWriteBack(_draw_target + fb_pos, segment * sizeof(uint16_t));

            pixels_remaining -= segment;
            src_offset += segment;
            fb_pos = (fb_pos + segment) % _fb_size;
        }
    }
}

void ESP32S3_Touch_LCD_7::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
    if (_draw_target == nullptr) return;
    // Bounds checking against padded framebuffer size (allows pre-drawing scroll content)
    if (x >= _fb_stride || y >= _fb_rows || x + w <= 0 || y + h <= 0) {
        return;
    }

    // Clip to padded framebuffer bounds
    int16_t x1 = max((int16_t)0, x);
    int16_t y1 = max((int16_t)0, y);
    int16_t x2 = min((int16_t)(_fb_stride - 1), (int16_t)(x + w - 1));
    int16_t y2 = min((int16_t)(_fb_rows - 1), (int16_t)(y + h - 1));

    int16_t clipped_w = x2 - x1 + 1;
    int16_t clipped_h = y2 - y1 + 1;

    // Fill pixels using linear addressing
    for (int16_t row = 0; row < clipped_h; row++) {
        // Linear position in ring buffer
        uint32_t fb_pos = (_pending_scroll_offset + (y1 + row) * _fb_stride + x1) % _fb_size;

        // Each row is contiguous - only need to handle wrap at buffer end
        int16_t pixels_remaining = clipped_w;

        while (pixels_remaining > 0) {
            int pixels_to_wrap = _fb_size - fb_pos;
            int segment = (pixels_remaining < pixels_to_wrap) ? pixels_remaining : pixels_to_wrap;

            uint16_t* dst = _draw_target + fb_pos;
            for (int i = 0; i < segment; i++) {
                dst[i] = color;
            }
            cacheWriteBack(dst, segment * sizeof(uint16_t));

            pixels_remaining -= segment;
            fb_pos = (fb_pos + segment) % _fb_size;
        }
    }
}

void ESP32S3_Touch_LCD_7::fillScreen(uint16_t color) {
    fillRect(0, 0, _width, _height, color);
}

void ESP32S3_Touch_LCD_7::drawPixel(int16_t x, int16_t y, uint16_t color) {
    if (_draw_target == nullptr) return;
    // Bounds checking against padded framebuffer size (allows pre-drawing scroll content)
    if (x < 0 || x >= _fb_stride || y < 0 || y >= _fb_rows) {
        return;
    }

    // Linear position in ring buffer
    uint32_t fb_pos = (_pending_scroll_offset + y * _fb_stride + x) % _fb_size;

    uint16_t* pixel = _draw_target + fb_pos;
    *pixel = color;

    // Flush this cache line (align to 64-byte boundary for PSRAM)
    uint32_t addr = (uint32_t)pixel & ~63;
    Cache_WriteBack_Addr(addr, 64);
}

void ESP32S3_Touch_LCD_7::flush() {
    if (_framebuffer == nullptr) return;
    // Write back entire framebuffer row by row
    const size_t row_bytes = _fb_stride * sizeof(uint16_t);
    for (int row = 0; row < _fb_rows; row++) {
        cacheWriteBack(_framebuffer + row * _fb_stride, row_bytes);
    }
}

bool ESP32S3_Touch_LCD_7::scrollFramebuffer(int16_t dx, int16_t dy, uint16_t fill_color) {
    if (!_framebuffer) {
        return false;
    }

    // Note: This function performs memory-copy scrolling (legacy).
    // For hardware scrolling without memory copies, use scroll() instead.

    // Vertical scrolling using SRAM staging buffer (dx == 0)
    if (dx == 0 && dy != 0) {
        // Allocate staging buffer on stack (full stride * 2 bytes)
        uint16_t line_buffer[_fb_stride];

        int16_t src_row_start;
        int16_t dst_row_start;
        int16_t num_rows_to_copy;

        if (dy < 0) {
            // Scroll up: move rows [abs(dy)...height-1] to [0...height-abs(dy)-1]
            src_row_start = -dy;
            dst_row_start = 0;
            num_rows_to_copy = _height - (-dy);
        } else {
            // Scroll down: move rows [0...height-dy-1] to [dy...height-1]
            src_row_start = 0;
            dst_row_start = dy;
            num_rows_to_copy = _height - dy;
        }

        // Copy rows line by line using SRAM staging
        // Copy direction matters for overlapping regions
        if (dy > 0) {
            // Scrolling down: copy from bottom to top to avoid overwriting source
            for (int16_t i = num_rows_to_copy - 1; i >= 0; i--) {
                uint16_t* src_line = _framebuffer + (src_row_start + i) * _fb_stride;
                uint16_t* dst_line = _framebuffer + (dst_row_start + i) * _fb_stride;

                // PSRAM -> SRAM -> PSRAM (copy visible width only)
                memcpy(line_buffer, src_line, _width * 2);
                memcpy(dst_line, line_buffer, _width * 2);
                cacheWriteBack(dst_line, _width * 2);
            }
        } else {
            // Scrolling up: copy from top to bottom to avoid overwriting source
            for (int16_t i = 0; i < num_rows_to_copy; i++) {
                uint16_t* src_line = _framebuffer + (src_row_start + i) * _fb_stride;
                uint16_t* dst_line = _framebuffer + (dst_row_start + i) * _fb_stride;

                // PSRAM -> SRAM -> PSRAM (copy visible width only)
                memcpy(line_buffer, src_line, _width * 2);
                memcpy(dst_line, line_buffer, _width * 2);
                cacheWriteBack(dst_line, _width * 2);
            }
        }

        return false;  // Fill not performed - caller should handle exposed area
    }

    // Horizontal or diagonal scrolling - copy line by line with stack-allocated staging buffer
    // Calculate source and destination regions
    int16_t src_x = (dx > 0) ? 0 : -dx;
    int16_t src_y = (dy > 0) ? 0 : -dy;
    int16_t dst_x = (dx > 0) ? dx : 0;
    int16_t dst_y = (dy > 0) ? dy : 0;
    int16_t copy_width = _width - abs(dx);
    int16_t copy_height = _height - abs(dy);

    // Allocate staging buffer on stack (visible width * 2 bytes)
    uint16_t line_buffer[_width];

    // Pre-fill the line buffer with fill_color once
    // This way we only read pixels that will actually move, and write a complete line
    for (int16_t x = 0; x < _width; x++) {
        line_buffer[x] = fill_color;
    }

    // Copy line by line using optimized PSRAM->SRAM->PSRAM
    if (dy > 0) {
        // Scrolling down: copy from bottom to top
        for (int16_t y = copy_height - 1; y >= 0; y--) {
            uint16_t* src_line = _framebuffer + (src_y + y) * _fb_stride + src_x;
            uint16_t* dst_line = _framebuffer + (dst_y + y) * _fb_stride;

            if (dx > 0) {
                // Scrolling right: read pixels directly to their final position in buffer
                memcpy(line_buffer + dx, src_line, copy_width * 2);
                // Left side (0 to dx-1) already has fill_color from pre-fill
            } else if (dx < 0) {
                // Scrolling left: read pixels directly to their final position
                memcpy(line_buffer, src_line, copy_width * 2);
                // Right side (copy_width to _width-1) already has fill_color from pre-fill
            } else {
                // No horizontal scroll: just copy pixels
                memcpy(line_buffer, src_line, copy_width * 2);
            }

            // Write complete line from SRAM to PSRAM (one burst write)
            memcpy(dst_line, line_buffer, _width * 2);
            cacheWriteBack(dst_line, _width * 2);
        }

        // Fill the top exposed rows with fill_color
        for (int16_t y = 0; y < dy; y++) {
            uint16_t* fill_line = _framebuffer + y * _fb_stride;
            memcpy(fill_line, line_buffer, _width * 2);  // Use pre-filled buffer
            cacheWriteBack(fill_line, _width * 2);
        }
    } else {
        // Scrolling up or no vertical scroll: copy from top to bottom
        for (int16_t y = 0; y < copy_height; y++) {
            uint16_t* src_line = _framebuffer + (src_y + y) * _fb_stride + src_x;
            uint16_t* dst_line = _framebuffer + (dst_y + y) * _fb_stride;

            if (dx > 0) {
                // Scrolling right: read pixels directly to their final position in buffer
                memcpy(line_buffer + dx, src_line, copy_width * 2);
                // Left side (0 to dx-1) already has fill_color from pre-fill
            } else if (dx < 0) {
                // Scrolling left: read pixels directly to their final position
                memcpy(line_buffer, src_line, copy_width * 2);
                // Right side (copy_width to _width-1) already has fill_color from pre-fill
            } else {
                // No horizontal scroll: just copy pixels
                memcpy(line_buffer, src_line, copy_width * 2);
            }

            // Write complete line from SRAM to PSRAM (one burst write)
            memcpy(dst_line, line_buffer, _width * 2);
            cacheWriteBack(dst_line, _width * 2);
        }

        // Fill the bottom exposed rows with fill_color
        if (dy < 0) {
            for (int16_t y = _height + dy; y < _height; y++) {
                uint16_t* fill_line = _framebuffer + y * _fb_stride;
                memcpy(fill_line, line_buffer, _width * 2);  // Use pre-filled buffer
                cacheWriteBack(fill_line, _width * 2);
            }
        }
    }

    return true;  // Fill was performed during scroll
}

void ESP32S3_Touch_LCD_7::setScrollOffset(int16_t x, int16_t y) {
    // Compute linear offset from x,y coordinates
    // Normalize to positive values within buffer bounds
    int32_t offset = y * _fb_stride + x;
    // Handle negative values by adding fb_size until positive
    while (offset < 0) offset += _fb_size;
    _pending_scroll_offset = offset % _fb_size;
}

void ESP32S3_Touch_LCD_7::scroll(int16_t dx, int16_t dy, uint16_t fill_color, bool fill) {
    if (_framebuffer == nullptr) return;
    if (dx == 0 && dy == 0) return;

    // Calculate new scroll offset: add linear delta to current offset
    // delta = dx + dy * fb_stride (horizontal + vertical contribution)
    int32_t delta = dx + dy * (int32_t)_fb_stride;
    int32_t new_offset = (int32_t)_pending_scroll_offset + delta;

    // Normalize to positive value within buffer
    while (new_offset < 0) new_offset += _fb_size;
    new_offset = new_offset % _fb_size;

    // Track how much we can pre-draw vs post-draw
    int16_t abs_dy = (dy > 0) ? dy : -dy;
    int16_t abs_dx = (dx > 0) ? dx : -dx;
    int16_t predraw_dy = (abs_dy <= _fb_pad_y) ? abs_dy : _fb_pad_y;
    int16_t predraw_dx = (abs_dx <= _fb_pad_x) ? abs_dx : _fb_pad_x;
    int16_t postdraw_dy = abs_dy - predraw_dy;
    int16_t postdraw_dx = abs_dx - predraw_dx;

    // Pre-draw fill into padding area (before changing scroll offset)
    // This writes to framebuffer positions that will be visible after scroll
    if (fill && (predraw_dy > 0 || predraw_dx > 0)) {
        // Pre-fill vertical exposed area
        if (predraw_dy > 0) {
            int16_t screen_y_base = (dy > 0) ? (_height - predraw_dy) : 0;
            for (int16_t row = 0; row < predraw_dy; row++) {
                int16_t screen_y = screen_y_base + row;
                uint32_t fb_pos = (new_offset + screen_y * _fb_stride) % _fb_size;
                for (int16_t x = 0; x < _width; x++) {
                    _framebuffer[(fb_pos + x) % _fb_size] = fill_color;
                }
                cacheWriteBack(_framebuffer + fb_pos, _width * sizeof(uint16_t));
            }
        }

        // Pre-fill horizontal exposed area
        if (predraw_dx > 0) {
            int16_t screen_x_base = (dx > 0) ? (_width - predraw_dx) : 0;
            for (int16_t y = 0; y < _height; y++) {
                uint32_t fb_pos = (new_offset + y * _fb_stride + screen_x_base) % _fb_size;
                for (int16_t col = 0; col < predraw_dx; col++) {
                    _framebuffer[(fb_pos + col) % _fb_size] = fill_color;
                }
                cacheWriteBack(_framebuffer + fb_pos, predraw_dx * sizeof(uint16_t));
            }
        }
    }

    // Update pending scroll offset (takes effect at next frame start)
    _pending_scroll_offset = new_offset;

    // Post-draw any remainder that didn't fit in padding
    if (fill && (postdraw_dy > 0 || postdraw_dx > 0)) {
        if (postdraw_dy > 0) {
            if (dy > 0) {
                // Scrolling down - fill remaining bottom rows
                fillRect(0, _height - abs_dy, _width, postdraw_dy, fill_color);
            } else {
                // Scrolling up - fill remaining top rows (after the pre-drawn ones)
                fillRect(0, predraw_dy, _width, postdraw_dy, fill_color);
            }
        }

        if (postdraw_dx > 0) {
            if (dx > 0) {
                // Scrolling right - fill remaining right columns
                fillRect(_width - abs_dx, 0, postdraw_dx, _height, fill_color);
            } else {
                // Scrolling left - fill remaining left columns (after the pre-drawn ones)
                fillRect(predraw_dx, 0, postdraw_dx, _height, fill_color);
            }
        }
    }
}

// Sprite API implementations

void ESP32S3_Touch_LCD_7::setSprite(uint8_t slot, const uint16_t* data, int16_t x, int16_t y,
                                uint16_t w, uint16_t h, uint16_t transparent,
                                uint32_t flags) {
    if (slot >= SPRITE_MAX_COUNT) return;

    // Temporarily mark as disabled while updating to ensure atomic visibility
    _pending_sprites[slot].yx = SPRITE_DISABLED;

    // Set all properties
    _pending_sprites[slot].data = data;
    _pending_sprites[slot].hw = SPRITE_HW(h, w);
    _pending_sprites[slot].transparent = transparent;
    _pending_sprites[slot].flags = flags;

    // Enable sprite by setting valid coordinates (do this last)
    _pending_sprites[slot].yx = SPRITE_YX(y, x);
}

void ESP32S3_Touch_LCD_7::clearSprite(uint8_t slot) {
    if (slot >= SPRITE_MAX_COUNT) return;
    _pending_sprites[slot].yx = SPRITE_DISABLED;
}

void ESP32S3_Touch_LCD_7::clearAllSprites() {
    for (int i = 0; i < SPRITE_MAX_COUNT; i++) {
        _pending_sprites[i].yx = SPRITE_DISABLED;
    }
}

bool ESP32S3_Touch_LCD_7::checkSpriteCollision(uint8_t slot) {
    if (slot >= SPRITE_MAX_COUNT) return false;

    // Check if collision flag is set and clear it atomically
    if (_sprite_collisions[slot]) {
        _sprite_collisions[slot] = 0;
        return true;
    }
    return false;
}

void ESP32S3_Touch_LCD_7::beginFlashMessage(int16_t y) {
    // Already active or pending -- don't double-allocate
    if (s_flashMsgPending != nullptr) return;

    // y == -1: blank screen only, no message text.
    // Use the sentinel non-null value so the ISR enters the flash overlay path
    // (which fills black first) but finds no message rows to blit.
    if (y == -1) {
        s_flashMsgW = 0;
        s_flashMsgH = 0;
        s_flashMsgStride = 0;
        s_flashMsgX = 0;
        s_flashMsgY = 0;
        s_flashMsgTile = false;
        __asm__ __volatile__("memw" ::: "memory");
        s_flashMsgPending = &s_blankSentinel;
    } else {
        int w = 0, h = 0, stride = 0;
        uint8_t* buf = renderMessageTo1bpp(s_flashWriteMsg, s_flashWriteMsgLen, 2, &w, &h, &stride);
        if (!buf) return;

        int bounceLines = _bounce_buffer_size_px / _width;  // 20 for 800px wide
        int msgY;
        bool tile = false;

        if (y == -3) {
            // Tile: message appears in every bounce buffer band, so it's visible
            // even if the DMA replays a single chunk during a flash stall.
            // msgY is the offset within each band; center the text vertically.
            msgY = (bounceLines - h) / 2;
            if (msgY < 0) msgY = 0;
            tile = true;
        } else if (y == -2) {
            // Default: center vertically, snapped to bounce buffer boundary
            msgY = ((_height - h) / 2);
            msgY = (msgY / bounceLines) * bounceLines;
        } else {
            msgY = (int)y;
            // Round down to bounce buffer boundary
            msgY = (msgY / bounceLines) * bounceLines;
            // Clamp to screen
            if (msgY < 0) msgY = 0;
            if (msgY + h > _height) msgY = ((_height - h) / bounceLines) * bounceLines;
        }

        s_flashMsgW = w;
        s_flashMsgH = h;
        s_flashMsgStride = stride;
        s_flashMsgX = (_width - w) / 2;
        s_flashMsgY = msgY;
        s_flashMsgTile = tile;

        __asm__ __volatile__("memw" ::: "memory");
        s_flashMsgPending = buf;
    }

    // Block until the ISR has applied the change (next frame boundary),
    // then wait for one more complete frame to ensure every bounce buffer
    // chunk has been rendered in flash-overlay mode before we return.
    // This prevents the caller's flash write from starting while the ISR
    // is still mid-frame reading from PSRAM.
    while (s_flashMsgActive != s_flashMsgPending) {
        vTaskDelay(1);
    }
    // Wait for a full frame: at minimum 24 ISR calls (480 lines / 20 lines per chunk).
    // We detect a full frame by waiting for pos_px == 0 to occur again, but since
    // we can't easily observe pos_px from here, we use a simple frame counter.
    uint32_t startCount = g_isrCallCount;
    while ((g_isrCallCount - startCount) < 25) {
        vTaskDelay(1);
    }
}

void ESP32S3_Touch_LCD_7::endFlashMessage() {
    // Save pointer for freeing after the ISR stops using it
    uint8_t* buf = (uint8_t*)s_flashMsgPending;
    if (!buf) return;

    // Clear pending -- the ISR will copy null to active at the next frame start
    s_flashMsgPending = nullptr;

    // Block until the ISR has applied the change (next frame boundary)
    while (s_flashMsgActive != nullptr) {
        vTaskDelay(1);
    }

    // Now safe to free -- the ISR is no longer referencing the buffer.
    // Don't free the static blank sentinel used by beginFlashMessage(-1).
    if (buf != &s_blankSentinel) {
        heap_caps_free(buf);
    }
}
