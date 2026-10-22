/**
 * @file  doom_port.h
 * @brief Between the Doom app (GUI task) and the Doom engine's task.
 *
 * The engine runs in a task of its own, created the first time the app is
 * opened. While the app is on screen it runs: frames go straight to the
 * display (video mode), sound to the headphones, and touches become key and
 * mouse events. Leaving the app pauses it at the next frame; opening it again
 * continues the game. The engine cannot be started a second time: once it has
 * ended (quit from its menu, or a fatal error), the board must be restarted.
 *
 * Screen layout (800 x 480): the game, doubled to 640 x 400, in the middle;
 * the control buttons in the 80-pixel strips left and right of it. The left
 * half of the game area is a joystick (drag from where you touch: forward,
 * back, strafe), the right half turns (drag) and fires (tap, or hold still).
 */
#ifndef DOOM_PORT_H
#define DOOM_PORT_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DOOM_DIR       "/doom"           /* the IWAD, configuration and saved games */
#define DOOM_ZONE_KB   4096              /* the engine's zone memory (Chocolate Doom's minimum) */

#define DOOM_SCREEN_W  800
#define DOOM_SCREEN_H  480
#define DOOM_GAME_X    80                /* the doubled 320 x 200 picture */
#define DOOM_GAME_Y    40
#define DOOM_GAME_W    640
#define DOOM_GAME_H    400
#define DOOM_STRIP_W   80                /* button strips left and right */

typedef enum
{
  DOOM_BTN_MENU,
  DOOM_BTN_MAP,
  DOOM_BTN_ENTER,
  DOOM_BTN_HOME,
  DOOM_BTN_USE,
  DOOM_BTN_WEAPON,
  DOOM_BTN_FIRE,
  DOOM_BTN_COUNT,
} doom_button_t;

typedef struct
{
  int16_t     x, y, w, h;      /**< on the screen */
  const char *label;
} doom_button_rect_t;

extern const doom_button_rect_t doom_buttons[DOOM_BTN_COUNT];

/** Pictures of the two button strips (DOOM_STRIP_W x DOOM_SCREEN_H, RGB565):
 *  [strip: 0 left, 1 right][0 normal, 1 pressed] */
typedef const uint16_t *doom_strips_t[2][2];

typedef enum
{
  DOOM_NOT_STARTED,
  DOOM_RUNNING,
  DOOM_PAUSED,
  DOOM_ENDED,
} doom_state_t;

/** Set up the engine's variables in SDRAM: main(), right after BSP_SDRAM_Init(). */
void doom_memory_init(void);

/* ---- GUI task -------------------------------------------------------------- */

/** True if a game file (IWAD) is in DOOM_DIR on the SD card. */
bool doom_find_iwad(void);

/** The button strips to show (kept, not copied). */
void doom_set_strips(const doom_strips_t *strips);

/** Start or continue the game. The display is in video mode and the audio
 *  output is free; @p volume 0..100. */
void doom_resume(uint8_t volume);

/** Pause; returns once the engine no longer draws or plays. */
void doom_pause(void);

/** The player asked to leave (home button) or the game ended. */
bool doom_wants_out(void);

doom_state_t doom_state(void);

/** Why the game ended (an error message), or NULL (quit from its menu). */
const char *doom_end_message(void);

/* ---- Doom task (the port's i_*.c files) ------------------------------------ */

/** The game ended: tell the app and stop the task for good. */
void doom_port_ended(const char *message) __attribute__((noreturn));

/** Wait here while the app has asked for a pause. */
void doom_port_checkpoint(void);

/** A frame buffer to draw the next frame into, with the buttons drawn. */
uint16_t *doom_port_frame(void);

/** Show @p frame (from doom_port_frame()). */
void doom_port_present(uint16_t *frame);

/** Headphone output on (@p volume 0..100) or off (i_sound.c). */
void I_SoundOutput(bool on, uint8_t volume);

/** The game was paused for @p ticks (i_timer.c: the clock leaves them out). */
void I_TimerPaused(uint32_t ticks);

/** Turn the touches into events for the engine (each tic). */
void doom_port_input(void);

#ifdef __cplusplus
}
#endif

#endif /* DOOM_PORT_H */
