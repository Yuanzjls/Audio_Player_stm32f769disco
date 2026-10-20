# STM32F769 Audio (wav) Player

This is a project based on STM32F769-Disco board, and implement a basic music player and buttons to control how to play the music. This project uses the STemWin and is add-ons, FreeRTOS is also involved to control the tasks and makes whole project efficiency. This project uses a customized font in SD card, so it is able to display Chinese songs' title and without worry about flash storage. The project right now only support for wav files (but all rates supports).

## How to

Copy /SD/YaHei.xbf to your SD card rootfile, and put *.wav songs to /Music fold. You can use any audio transfer software to make wav files and put them into /Music. 

## Build (CMake)

Requirements: CMake ≥ 3.20, Ninja, [Arm GNU Toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads) (`arm-none-eabi-gcc` on PATH).
On Windows: `winget install Kitware.CMake Ninja-build.Ninja Arm.GnuArmEmbeddedToolchain xpack-dev-tools.openocd-xpack`

STemWin is a closed binary library that is not in this repo. Download the STM32CubeF7 package from
[st.com](https://www.st.com/en/embedded-software/stm32cubef7.html) (STemWin 5.44, CubeF7 v1.15/v1.16) and copy into `STemWin/Lib/`:

* `STemWin_CM7_OS_wc32_ot_ARGB.a` (from `Middlewares/ST/STemWin/Lib`)
* `STemWin_Addons_CM7_wc32.a` (STemWin add-ons, provides `SLIDER_SetSTSkin`)

Or pass `-DSTEMWIN_LIB=<path>` / `-DSTEMWIN_ADDONS_LIB=<path>` at configure time.

```sh
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
* Serial monitor: ST-LINK virtual COM port, auto-detected, **9600 baud 7O1** (the firmware's `UART_WORDLENGTH_8B` includes the odd-parity bit).
  From a terminal: `powershell -ExecutionPolicy Bypass -File tools/serial_monitor.ps1 [-Port COM8]`
* `Ctrl+Shift+B` – build (Debug)
* `Ctrl+Shift+P` → **Tasks: Run Task** → *Flash (Debug)*, *Flash (Release)*, *Flash via USB drive*, *Rebuild*, *Clean*, *Reset board*
* `F5` – build, flash and stop at `main()`. Pick *Debug (OpenOCD + GDB)* (C/C++ extension only) or
  *Debug (Cortex-Debug)* (with the [Cortex-Debug](https://marketplace.visualstudio.com/items?itemName=marus25.cortex-debug) extension: register/peripheral/FreeRTOS views).

## Demo video

https://www.youtube.com/watch?v=nAqI61Whyyc&feature=youtu.be

## Todo

* Add other formats (such as MP3 and flac) support.
* Add lyric support.
* Add frequency spectrum