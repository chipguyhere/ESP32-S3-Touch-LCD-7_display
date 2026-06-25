# chipguy_ESP32S3_Touch_LCD_7_display

A self-contained **display + touch** driver for the **Waveshare
ESP32-S3-Touch-LCD-7** — the 7" board with an **800×480 ST7262 RGB LCD** and a
**GT911 capacitive touch** panel. It drives both so you don't have to think
about either: the hardware is fixed and fully handled, leaving you to write an
**LVGL 9** app and nothing else.

The bundled examples are the heart of the library. The two **starting points**
are complete and working — the display, touch, and LVGL are already wired up and
running before your code gets control. You pick the one that matches how you want
to build your UI, copy it, and start replacing its placeholder UI with your own.
A third example, **LvglCalculator**, is a finished app you can read for ideas.

> **Scope:** this library covers the **display and touch** only. The board also
> carries a microSD slot, an RS-485 / CAN transceiver, and other peripherals;
> those are not driven here. See Waveshare's demo for those peripherals.

---

## Installation

This library is distributed by downloading the code as a ZIP from GitHub —
there is no Arduino Library Manager entry. To install:

1. On the GitHub project page, click **Code ▸ Download ZIP**.
2. Unzip it, and move the resulting folder into your Arduino **libraries**
   directory:
   - macOS / Linux: `~/Documents/Arduino/libraries/`
   - Windows: `Documents\Arduino\libraries\`
3. Restart Arduino IDE so the examples will appear in the File menu.

(Alternatively, in the IDE: **Sketch ▸ Include Library ▸ Add .ZIP Library…**
and select the downloaded ZIP.)

### Dependency: the board library

This library depends on **`chipguy_ESP32-S3-Touch-LCD-7_board`**, which owns the board's shared I²C bus
and CH422G expander. Install it into your `libraries/` directory the same way.
(See "The CH422G I²C expander" below.)

### Dependency: LVGL 9

The examples need **LVGL 9**. Install it via the Arduino **Library Manager**
(search "lvgl"). Each example sketch folder ships its own `lv_conf.h`; LVGL reads
its configuration from that file, so keep it next to the `.ino`.

This library was developed against LVGL 9.3 (the current version of LVGL that is
compatible with SquareLine) but is very likely compatible with later versions.

### Arduino IDE board settings

This board uses a generic ESP32-S3 board definition. In **Tools**, set:

- **Board:** ESP32S3 Dev Module
- **PSRAM:** OPI PSRAM  *(required — the framebuffers live in PSRAM)*
- **Flash Size:** 8 MB (64 Mbit)
- **USB CDC On Boot:** Enabled  *(required to see serial output)*
- **Upload Speed:** 460800

> **Serial note:** USB serial on this board is **`Serial0`**, not `Serial`. With
> "USB CDC On Boot" enabled, `Serial0.begin(115200)` prints to the USB port.

---

## The two examples — pick your starting point

Open them from **File ▸ Examples ▸ chipguy_ESP32-S3-Touch-LCD-7_display**. Both
come up running on the hardware immediately, so you can confirm your board works
before you change a line. They differ only in **how you author the UI**.

To turn either one into your own project, **save it under a new name**
(File ▸ Save As) so your work lives outside the library, then start editing.
Each example folder is self-contained — it carries its own `lv_setup.hpp` and
`lv_conf.h` — so a copied sketch is fully standalone.

### LvglClaudeCodeStub — write the UI yourself (by hand or with an AI)

Use this when you want to build the interface **in code**: directly, or by
handing the sketch to an AI coding assistant such as Claude Code.

The UI lives in a tiny `ui.h` / `ui.cpp` pair in the sketch folder. All the
hardware/LVGL setup happens in the sketch before `ui_init()` is called, so
`ui_init()` is a blank canvas: it receives a live LVGL screen and you create
widgets on it. To start your app:

1. Open `ui.cpp` and look at `ui_init()`. The demo it ships with (a label that
   follows your finger, plus a color-cycling rectangle) is just there to prove
   the display and touch work — **delete it.**
2. In its place, build your own UI on `lv_screen_active()` using normal LVGL
   calls (`lv_label_create`, `lv_button_create`, event callbacks, …).
3. As your app grows, add more screens and helper functions, and declare the
   ones the sketch needs in `ui.h`.

**Reach for this one** when you want full control, a tiny footprint, or
AI-assisted/hand-written UI code.

### LvglCalculator — a worked, resolution-independent example

A working four-function calculator styled after the dark iOS calculator. Its
entire layout — key diameter, gaps, margins, and font sizes — is derived from
the live display resolution at runtime, so nothing is pinned to 800×480. It's a
good read for how to build a UI in plain LVGL. Not a blank starting point — copy
it if you want a calculator, or just study `build_keypad()` in `ui.cpp`. (This
example is adapted from the rectangular ESP32-S3-Touch-LCD-2 version, since the
calculator's grid layout suits a rectangular panel.)

### Lvgl93SquarelineLauncher — design the UI visually in SquareLine Studio

Use this when you'd rather **design the interface visually** in
[SquareLine Studio](https://squareline.io/) (a drag-and-drop LVGL UI editor) and
just run the result on the board.

The sketch's `src/` folder holds a small **placeholder** UI in the exact file
shape a SquareLine export produces (`ui.c`, `ui_Screen1.c`, …). The sketch calls
the `ui_init()` that SquareLine generates. To start your app:

1. In SquareLine Studio, create a project sized **800×480**, **16-bit** color,
   with **no rotation or offset**.
2. Lay out your screens visually.
3. **Export ▸ UI files** (the "Arduino TFT_eSPI profile" works, among others).
4. **Replace the whole `src/` folder** with your export. Build and upload —
   `ui_init()` now runs your design.

Treat `src/` as disposable: you overwrite it wholesale each time you re-export
from SquareLine. Your own application logic lives in the **sketch** — after
calling `ui_init()`, attach event handlers from the `.ino` to the UI objects
SquareLine exposes (e.g. `ui_Button1`).

**Reach for this one** when you want to iterate on layout visually and write
little or no C for the UI itself.

---

## What the examples give you: the `lv_setup` API

Both examples wire the hardware to LVGL through one small header,
**`lv_setup.hpp`** (plus **`lv_conf.h`** for LVGL's build config). That header,
and the global objects it defines, is the entire surface you interact with — and
once `begin()` returns, you are just using plain LVGL.

```cpp
#include "lv_conf.h"
#include "lvgl.h"
#include "lv_setup.hpp"

void setup() {
    Serial0.begin(115200);

    lv_setup.begin();           // initialize display + touch + LVGL

    // ...build your UI on lv_screen_active() here (or call ui_init())...
}

void loop() {
    lv_timer_handler();         // let LVGL run
    delay(5);
}
```

What the header provides:

- **`lv_setup.begin()`** — call once in `setup()`. Brings up the ST7262 display,
  the GT911 touch, and LVGL; allocates the two PSRAM framebuffers, creates the
  LVGL display in **direct render mode** plus a pointer (touch) input device, and
  connects them. It also drives the onboard CH422G I²C expander to release the
  display and touch resets and switch the backlight on — you don't touch any of
  that. After this returns, the active LVGL screen exists and you build your UI
  with ordinary LVGL calls.
- **`lv_setup.begin(180)`** — same, but for a panel **mounted upside-down**.
  Pass `180` and both the image and the touch are flipped 180° for you (the
  display flip happens in the driver, still reading PSRAM forward/sequentially, so
  there's no meaningful performance cost; sprites and the flash-write overlay flip
  too). `0` (the default) is normal orientation.
- **`lv_setup.begin(90)` / `lv_setup.begin(270)`** — **portrait** (480×800).
  90° and 270° can't be done in the driver's framebuffer read without slow
  vertical-stripe access, so they're handled one level up: LVGL renders in
  **PARTIAL** mode in portrait, and the flush callback rotates each changed
  region into the landscape framebuffers, keeping the two buffers in sync with a
  deferred dirty-rectangle scheme (it never copies pixels that the next frame is
  about to overwrite — e.g. while scrolling). The driver itself still reads PSRAM
  forward. Touch is mapped to match. This mode costs one extra **internal-SRAM**
  draw band (≈80 KB, `LV_SETUP_ROT_BAND_BYTES` in `lv_setup.hpp`); a bigger band
  means longer contiguous PSRAM writes during the rotate. In portrait,
  `lv_display_get_horizontal_resolution()` reports 480 and the vertical 800. If
  the image/touch come out the wrong way round, swap 90 and 270.

  *The index aliases `1`/`2`/`3` are accepted everywhere as `90`/`180`/`270` —
  e.g. `lv_setup.begin(1)` is the same as `lv_setup.begin(90)`.*
- **`display`** — a global `ESP32S3_Touch_LCD_7` display object. Handy methods:
  `display.width()`, `display.height()` (always the physical 800×480), and
  `display.setBacklight(true/false)` to switch the backlight on or off.

You don't render or flush anything yourself — LVGL renders straight into the
panel's framebuffers and the driver flips them at vsync (tear-free, double
buffered). Your job is to create LVGL widgets and call `lv_timer_handler()` in
`loop()`.

> **Backlight:** on this board the LCD backlight is wired to the CH422G expander,
> not a PWM-capable ESP32 GPIO, so it is **on/off only** — there is no brightness
> dimming. `setBacklight()` accepts a percentage for API compatibility, but any
> value > 0 simply turns the backlight on.

> **Touch orientation:** the GT911 is configured to report coordinates aligned
> with the panel, so this driver passes them straight through (matching
> Waveshare's own demo). If you find touch mirrored or rotated relative to what
> you see, adjust the mapping in `lv_setup.hpp`'s `_touchpad_read`, or call
> `setRotation()` on the touch object.

### Portable across chipguy board libraries

Because every hardware detail lives in just two files — **`lv_setup.hpp`** and
**`lv_conf.h`** — your application code (`ui.cpp`, or the SquareLine `src/`
files) is hardware-independent. It only ever talks to LVGL. To move an app to a
**different display board**, drop in the `lv_setup.hpp` and `lv_conf.h` from that
board's chipguy library, and the rest of your sketch is unchanged.

---

## License

This library is **mixed-license**:

- The original code (display driver, I²C/expander helper, examples) is **MIT** —
  see [LICENSE](LICENSE).
- The **GT911 touch driver** (`chipguy_ESP32S3_Touch_LCD_7_touch.h` / `.cpp`)
  is a derivative of [TAMC_GT911](https://github.com/tamctec/gt911-arduino)
  (Copyright © TAMC) and is licensed under the **Apache License, Version 2.0** —
  see [LICENSE-Apache-2.0.txt](LICENSE-Apache-2.0.txt). Those files carry the
  upstream attribution and note the modifications made for this board.

The ST7262 panel has no programmable registers — the RGB timings are standard
panel values (from Waveshare's ESP32-S3-Touch-LCD-7 demo). `lv_conf.h` is based
on LVGL's template (LVGL is MIT-licensed).
