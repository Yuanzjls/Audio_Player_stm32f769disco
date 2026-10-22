# Doom app (optional)

Chocolate Doom running as an app of the player, played with the touch screen.
It is **not part of the default firmware**: build it in with

```sh
cmake --preset Release -DWITH_DOOM=ON      # or: cmake -S . -B build/Doom -DWITH_DOOM=ON ...
cmake --build build/Release
```

## Licence

Everything in this folder is licensed under the **GNU General Public License,
version 2 or later** (see [COPYING](COPYING)): the Doom engine is
Copyright (C) 1993-1996 id Software and (C) 2005-2014 Simon Howard and the
Chocolate Doom contributors; the port in `port/` is under the same licence.
A firmware built with `-DWITH_DOOM=ON` contains this code, so whoever
distributes such a firmware must do so under the GPL (with its source code).
The default firmware contains none of it.

## Game data

Doom needs an IWAD: copy `DOOM1.WAD` (the free shareware episode), or the
retail `DOOM.WAD` / `DOOM2.WAD`, into **`/doom`** on the SD card. The game keeps
its settings (`/doom/default.cfg`) and saved games (`/doom/savegames/`) there
too. No WAD is part of this repository.

## Playing

The game fills the middle of the screen (320 x 200 doubled to 640 x 400), with
two strips of buttons beside it:

| Left | Right |
|---|---|
| MENU (Esc) | Home: back to the home screen (the game pauses) |
| MAP (automap) | USE (doors, switches) |
| OK (Enter, and "y" in questions) | WPN (next weapon) |
| | FIRE |

- **Left half of the picture**: a joystick. Put a finger down anywhere and drag
  from there: up / down to walk, left / right to strafe (in the menus: move
  the selection, change a slider).
- **Right half of the picture**: drag left / right to turn; tap to fire once,
  or hold still for a moment to keep firing (and drag while firing to aim).
  In the menus, a tap selects.

The panel reads two fingers at a time: walk with one thumb and aim or fire
with the other. The game always runs.

Leaving the app pauses the game; opening it again continues it. The engine
cannot be started a second time: after quitting from the game's menu (or a
fatal error), restart the board to play again.

Sound effects and music play on the headphones (the music player stops
while Doom runs). The music is the game's own: its MUS songs played through
an emulated OPL2 (AdLib / Sound Blaster) chip, as on a 1990s PC. Saved games
and the music's temporary MIDI file (`doom.mid`) go to `/doom` too.

## How it is put together

| Part | Where |
|---|---|
| Engine | `chocdoom/`, `chocdoom/doom/`: Chocolate Doom 2.1.0 as in [Yuanzjls/Doom_STM32F769-DISCO](https://github.com/Yuanzjls/Doom_STM32F769-DISCO) (itself after [floppes/stm32doom](https://github.com/floppes/stm32doom)), Doom only (no Heretic / Hexen), without its platform files; with bug fixes from later Chocolate Doom (see below) |
| Music | `chocdoom/i_oplmusic.c`, `midifile.*`, `mus2mid.*` and `chocdoom/opl/` (the OPL library with the DOSBox OPL emulator, DBOPL) from Chocolate Doom 2.2.1, the last version with DBOPL (later ones use Nuked OPL3, too slow for this CPU) |
| C library | `port/doom_libc.*`: every engine file is compiled with `-include doom_libc.h`, which maps stdio to FatFs files, console output to the UART log and malloc to a heap of its own (TLSF, 512 KB in SDRAM) |
| System, timer | `port/i_system.c` (zone: 4 MB fixed in SDRAM; quitting or an error ends the engine's task instead of exiting), `port/i_timer.c` (FreeRTOS tick; paused time does not count) |
| Video | `port/i_video.c`: the palette picture doubled straight into the display's frame buffers (the video mode of `Board/Src/display.c`) |
| Sound | `port/i_sound.c`: a task woken by the audio DMA interrupt fills each 23 ms half of the buffer at 11025 Hz with the music and up to 16 effects (volume, stereo separation); `port/opl_stm32.c`: the OPL driver (after `opl_sdl.c`): DBOPL renders into that mix, and the music player's timed events run as the rendered audio reaches them |
| Task, input | `port/doom_port.c`: the engine's task (started and paused by the app), the button strips around the frames, touches to key and mouse events |
| App | `App/Src/app_doom.c`: the home screen tile, the button pictures (drawn once with LVGL), entering and leaving video mode |

Memory: the engine's own variables (about 300 KB) are placed in SDRAM by the
linker script (`.doom_data`, `.doom_bss`; set up by `doom_memory_init()` in
`main()`), next to the 4 MB zone and the 512 KB heap. LVGL's heap is 1 MB
smaller in Doom builds to make room.

Changes to the engine files:

- `doom/m_menu.c` reads the saved game names through stdio again (as
  upstream); `unsigned long` byte counts passed to FatFs are `UINT`; a
  missing `void` in `doom/r_data.c`.
- Fixes from later Chocolate Doom: undefined behaviour and overflows
  (`P_SubRandom`, false boolean assumptions, `G_CheckSpot`, `P_SpawnMapThing`,
  `A_Chase`, `R_StoreWallRange`), bounds and crashes (solidsegs, the switch
  list, `P_GiveAmmo`, `F_TextWrite`, DR doors on one-sided lines, REJECT
  padding), game bugs (`EV_BuildStairs` fields, mouse motion as menu keys).
  The commit messages name the upstream commits.
- For the music: `opl/opl.c` uses the board's driver and its own `OPL_Delay`
  (no SDL); `i_sound.h` has the OPL driver version type of 2.2.1;
  `i_swap.h` provides SDL's big-endian swaps; `m_misc.c` makes temporary
  files in `/doom`.

The platform files (`i_main`, `i_system`, `i_timer`, `i_video`, `i_sound`,
the unused `i_scale`, `i_videohr`, and `opl/opl_sdl.c`) are replaced by those
in `port/`.
