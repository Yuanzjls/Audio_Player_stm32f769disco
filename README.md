# STM32F769 Audio (wav) Player

A touch-screen music player for the STM32F769I-Discovery board. It plays `.wav` files from the
microSD card through the headphone jack, with play/pause, next, repeat-one, a seek bar and volume
control. The UI is built with [LVGL](https://lvgl.io) 9 and runs on FreeRTOS. Song titles in any
language (e.g. Chinese) are drawn with a TrueType font loaded from the SD card.

Supported files: PCM WAV, 16-bit, mono or stereo, at 8 / 11.025 / 16 / 22.05 / 32 / 44.1 / 48 / 96 kHz.

## SD card

```
/Music/*.wav      songs, played in directory order
/font.ttf         optional: font for the song titles (font.otf / font.ttc also work, for a
                  .ttc collection the first face is used), e.g. Noto Sans SC for Chinese.
                  Without it only Latin titles render.
```

## Build (CMake)

Requirements: CMake ≥ 3.20, Ninja, [Arm GNU Toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads) (`arm-none-eabi-gcc` on PATH).
On Windows: `winget install Kitware.CMake Ninja-build.Ninja Arm.GnuArmEmbeddedToolchain xpack-dev-tools.openocd-xpack`

LVGL is a git submodule:

```sh
git submodule update --init
cmake --preset Debug          # or Release
cmake --build build/Debug     # -> build/Debug/Audio_Player_stm32f769disco.elf/.hex/.bin
```

## Flash

* `cmake --build build/Debug --target flash` – STM32CubeProgrammer CLI if installed, otherwise OpenOCD.
  Requires the ST-LINK USB driver ([STSW-LINK009](https://www.st.com/en/development-tools/stsw-link009.html)) on Windows.
* `cmake --build build/Debug --target flash_drive` – copies the `.bin` to the board's `DIS_F769NI` USB drive
  (drag-and-drop programming, no driver needed). Override the drive with `-DFLASH_DRIVE=F:/`.

## VS Code

* Status bar buttons **Build | Flash | Monitor | Flash+Monitor | Reset** (via the recommended [Tasks](https://marketplace.visualstudio.com/items?itemName=actboy168.tasks) extension)
* Serial monitor: log output on the ST-LINK virtual COM port, auto-detected, **115200 8N1**.
  From a terminal: `powershell -ExecutionPolicy Bypass -File tools/serial_monitor.ps1 [-Port COM8]`
* `Ctrl+Shift+B` – build (Debug)
* `Ctrl+Shift+P` → **Tasks: Run Task** → *Flash (Debug)*, *Flash (Release)*, *Flash via USB drive*, *Rebuild*, *Clean*, *Reset board*
* `F5` – build, flash and stop at `main()`. Pick *Debug (OpenOCD + GDB)* (C/C++ extension only) or
  *Debug (Cortex-Debug)* (with the [Cortex-Debug](https://marketplace.visualstudio.com/items?itemName=marus25.cortex-debug) extension: register/peripheral/FreeRTOS views).

## Code layout

| Folder | Contents |
|---|---|
| `App/` | `player` (playlist + streaming task, command queue), `wav` (RIFF parser), `ui` (LVGL screen) |
| `Board/` | `audio_out` (SAI1 + WM8994), `display` (DSI/LTDC/OTM8009A as LVGL display), `touch` (FT6x06), `i2c_bus` (lock for the shared codec/touch I2C), `log` (UART) |
| `Core/` | start-up (`main.c`: MPU, caches, clocks, tasks), interrupt handlers, HAL/FreeRTOS/LVGL configuration |
| `Middlewares/lvgl` | LVGL 9.6 (submodule) |
| `Drivers/`, `FATFS/`, `FREERTOS/` | ST HAL/BSP, FatFs (with the SD-card glue in `FATFS/diskio.c`), FreeRTOS kernel |

Tasks: `player` (priority 3) streams audio; `gui` (priority 1) runs LVGL. The UI only talks to the
player through `player_*()` commands and reads a status snapshot, so the two never share state directly.

## Demo video

https://www.youtube.com/watch?v=nAqI61Whyyc&feature=youtu.be (STemWin version)

## Todo

* Add other formats (such as MP3 and flac) support.
* Add lyric support.
* Add frequency spectrum
