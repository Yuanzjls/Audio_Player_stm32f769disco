/**
 * @file  doom_port.c
 * @brief The Doom engine's task: started and paused by the Doom app, frames
 *        with the button strips around them, and touches turned into events.
 */
#include <string.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "d_event.h"
#include "doomkeys.h"
#include "doomstat.h"
#include "m_argv.h"
#include "m_controls.h"
#include "doom_port.h"   /* after the engine's headers: they define false and true */
#include "display.h"
#include "ff.h"
#include "log.h"
#include "touch.h"

#define TASK_STACK_WORDS 4096
#define TASK_PRIORITY    1            /* the GUI's: it keeps reading the touch screen */

#define BUTTON_SLOP      6            /* touches this close to a button count */
#define STICK_DEAD       24           /* joystick: pixels before a direction counts */
#define TRACK_RANGE      160          /* a finger moves at most this far between tics */
#define AIM_SLOP         10           /* right side: a drag, not a tap */
#define AIM_HOLD_MS      200          /* right side held still this long: fire */
#define TURN_SCALE       8            /* mouse units per pixel dragged */
#define TAP_FIRE_TICS    3
#define MAX_HELD         16

void D_DoomMain(void);
extern int messageToPrint;   /* m_menu.c: a question is on screen */

const doom_button_rect_t doom_buttons[DOOM_BTN_COUNT] = {
  [DOOM_BTN_MENU]   = {   6,  48, 68,  88, "MENU" },
  [DOOM_BTN_MAP]    = {   6, 148, 68,  88, "MAP" },
  [DOOM_BTN_ENTER]  = {   6, 248, 68,  88, "OK" },
  [DOOM_BTN_HOME]   = { 726,  48, 68,  88, "EXIT" },
  [DOOM_BTN_USE]    = { 726, 148, 68,  88, "USE" },
  [DOOM_BTN_WEAPON] = { 726, 248, 68,  88, "WPN" },
  [DOOM_BTN_FIRE]   = { 726, 348, 68, 108, "FIRE" },
};

static TaskHandle_t         s_task;
static SemaphoreHandle_t    s_resume;      /* engine waits on it while paused */
static SemaphoreHandle_t    s_parked;      /* given when the engine has stopped */
static volatile doom_state_t s_state = DOOM_NOT_STARTED;
static volatile bool        s_pause_wanted;
static volatile bool        s_wants_out;
static char                 s_end_text[200];
static const char          *s_end_message;
static const doom_strips_t *s_strips;
static uint16_t            *s_prepared[2];  /* frame buffers whose black bands are drawn */

/* ---- input state (engine task) --------------------------------------------- */

typedef struct
{
  bool     active;
  int32_t  x0, y0;            /* where the finger came down */
  int32_t  x, y;              /* where it is */
} finger_t;

typedef enum { AIM_UNDECIDED, AIM_TURN, AIM_FIRE } aim_mode_t;

static finger_t   s_stick;
static finger_t   s_aim;
static aim_mode_t s_aim_mode;
static int32_t    s_aim_turned_x;        /* drag position the turning has followed to */
static TickType_t s_aim_start;
static int        s_tap_tics;            /* a tap on the right: fire (or select) this many tics */
static bool       s_tap_select;
static uint32_t   s_buttons;             /* pressed buttons, bit per doom_button_t */
static int        s_held[MAX_HELD];      /* keys the engine was told are down */
static int        s_held_count;

/* ---- start-up ----------------------------------------------------------------- */

/* Linker script: the engine's initialised and zeroed variables, in SDRAM */
extern uint8_t __doom_data_start[], __doom_data_end[], __doom_data_load[];
extern uint8_t __doom_bss_start[], __doom_bss_end[];

void doom_memory_init(void)
{
  memcpy(__doom_data_start, __doom_data_load, (size_t)(__doom_data_end - __doom_data_start));
  memset(__doom_bss_start, 0, (size_t)(__doom_bss_end - __doom_bss_start));
}

/* ---- GUI side --------------------------------------------------------------- */

bool doom_find_iwad(void)
{
  static const char *const k_iwads[] = { "doom1.wad", "doom.wad", "doom2.wad", "plutonia.wad", "tnt.wad" };
  char path[48];
  FILINFO fi;
  for (size_t i = 0; i < sizeof(k_iwads) / sizeof(k_iwads[0]); i++)
  {
    snprintf(path, sizeof(path), DOOM_DIR "/%s", k_iwads[i]);
    if (f_stat(path, &fi) == FR_OK)
    {
      return true;
    }
  }
  return false;
}

void doom_set_strips(const doom_strips_t *strips)
{
  s_strips = strips;
}

static void doom_task(void *arg)
{
  static char *argv[] = { "doom", NULL };
  (void)arg;
  myargc = 1;
  myargv = argv;
  D_DoomMain();             /* the game loop: does not come back */
  doom_port_ended(NULL);
}

void doom_resume(uint8_t volume)
{
  if (s_state == DOOM_ENDED || s_state == DOOM_RUNNING)
  {
    return;
  }
  s_prepared[0] = s_prepared[1] = NULL;   /* LVGL drew into the frame buffers meanwhile */
  s_wants_out = false;
  I_SoundOutput(true, volume);

  if (s_task == NULL)
  {
    s_resume = xSemaphoreCreateBinary();
    s_parked = xSemaphoreCreateBinary();
    s_state  = DOOM_RUNNING;
    if (s_resume == NULL || s_parked == NULL ||
        xTaskCreate(doom_task, "doom", TASK_STACK_WORDS, NULL, TASK_PRIORITY, &s_task) != pdPASS)
    {
      LOG_ERROR("doom: task not created");
      s_end_message = "Not enough memory to start Doom.";
      s_state = DOOM_ENDED;
      s_wants_out = true;
    }
    return;
  }
  s_state = DOOM_RUNNING;
  xSemaphoreGive(s_resume);
}

void doom_pause(void)
{
  if (s_state == DOOM_RUNNING)
  {
    s_pause_wanted = true;
    xSemaphoreTake(s_parked, portMAX_DELAY);   /* at its next frame (or it ended) */
    if (s_state == DOOM_RUNNING)
    {
      s_state = DOOM_PAUSED;
    }
  }
  I_SoundOutput(false, 0);
  s_wants_out = false;
}

bool doom_wants_out(void)              { return s_wants_out; }
doom_state_t doom_state(void)          { return s_state; }
const char *doom_end_message(void)     { return s_end_message; }

/* ---- engine side ------------------------------------------------------------ */

void doom_port_ended(const char *message)
{
  if (message != NULL)
  {
    snprintf(s_end_text, sizeof(s_end_text), "%s", message);
    s_end_message = s_end_text;
    LOG_ERROR("doom: %s", message);
  }
  else
  {
    LOG_INFO("doom: quit");
  }
  s_state = DOOM_ENDED;
  s_wants_out = true;
  xSemaphoreGive(s_parked);    /* in case the app waits for a pause */
  for (;;)
  {
    vTaskSuspend(NULL);
  }
}

static void post_key(evtype_t type, int key)
{
  event_t ev;
  ev.type  = type;
  ev.data1 = key;
  ev.data2 = (key > 0 && key < 128) ? key : 0;   /* the typed character */
  ev.data3 = key;
  ev.data4 = 0;
  D_PostEvent(&ev);
}

/* Tell the engine all keys are up, forget the fingers */
static void release_all(void)
{
  for (int i = 0; i < s_held_count; i++)
  {
    post_key(ev_keyup, s_held[i]);
  }
  s_held_count = 0;
  s_stick.active = false;
  s_aim.active = false;
  s_buttons = 0;
  s_tap_tics = 0;
}

void doom_port_checkpoint(void)
{
  if (!s_pause_wanted)
  {
    return;
  }
  s_pause_wanted = false;
  release_all();
  const TickType_t start = xTaskGetTickCount();
  xSemaphoreGive(s_parked);
  xSemaphoreTake(s_resume, portMAX_DELAY);
  I_TimerPaused(xTaskGetTickCount() - start);
}

uint16_t *doom_port_frame(void)
{
  uint16_t *frame = display_video_frame();

  /* Black above and below the game, once per buffer since the last resume */
  if (frame != s_prepared[0] && frame != s_prepared[1])
  {
    for (int y = 0; y < DOOM_SCREEN_H; y++)
    {
      if (y < DOOM_GAME_Y || y >= DOOM_GAME_Y + DOOM_GAME_H)
      {
        memset(&frame[y * DOOM_SCREEN_W + DOOM_GAME_X], 0, DOOM_GAME_W * 2U);
      }
    }
    s_prepared[(s_prepared[0] == NULL) ? 0 : 1] = frame;
  }

  /* The button strips, pressed buttons from the second picture */
  for (int k = 0; k < 2; k++)
  {
    const int x0 = (k == 0) ? 0 : DOOM_SCREEN_W - DOOM_STRIP_W;
    const uint16_t *normal  = (s_strips != NULL) ? (*s_strips)[k][0] : NULL;
    const uint16_t *pressed = (s_strips != NULL) ? (*s_strips)[k][1] : NULL;
    for (int y = 0; y < DOOM_SCREEN_H; y++)
    {
      uint16_t *dst = &frame[y * DOOM_SCREEN_W + x0];
      if (normal != NULL)
      {
        memcpy(dst, &normal[y * DOOM_STRIP_W], DOOM_STRIP_W * 2U);
      }
      else
      {
        memset(dst, 0, DOOM_STRIP_W * 2U);
      }
    }
    for (int b = 0; b < DOOM_BTN_COUNT && pressed != NULL; b++)
    {
      const doom_button_rect_t *r = &doom_buttons[b];
      if ((s_buttons & (1U << b)) == 0U || r->x < x0 || r->x >= x0 + DOOM_STRIP_W)
      {
        continue;
      }
      for (int y = r->y; y < r->y + r->h; y++)
      {
        memcpy(&frame[y * DOOM_SCREEN_W + r->x], &pressed[y * DOOM_STRIP_W + (r->x - x0)], (size_t)r->w * 2U);
      }
    }
  }
  return frame;
}

void doom_port_present(uint16_t *frame)
{
  display_video_present(frame);
}

/* ---- touches -> events ------------------------------------------------------- */

static int32_t distance2(int32_t ax, int32_t ay, int32_t bx, int32_t by)
{
  return (ax - bx) * (ax - bx) + (ay - by) * (ay - by);
}

/* Follow a finger to the nearest unused touch; false if it was lifted */
static bool follow(finger_t *f, const touch_points_t *p, bool *used)
{
  int best = -1;
  int32_t best_d = TRACK_RANGE * TRACK_RANGE;
  for (int i = 0; i < p->count; i++)
  {
    const int32_t d = distance2(f->x, f->y, p->x[i], p->y[i]);
    if (!used[i] && d <= best_d)
    {
      best = i;
      best_d = d;
    }
  }
  if (best < 0)
  {
    return false;
  }
  used[best] = true;
  f->x = p->x[best];
  f->y = p->y[best];
  return true;
}

static int button_at(int32_t x, int32_t y)
{
  for (int b = 0; b < DOOM_BTN_COUNT; b++)
  {
    const doom_button_rect_t *r = &doom_buttons[b];
    if (x >= r->x - BUTTON_SLOP && x < r->x + r->w + BUTTON_SLOP &&
        y >= r->y - BUTTON_SLOP && y < r->y + r->h + BUTTON_SLOP)
    {
      return b;
    }
  }
  return -1;
}

static void want(int *keys, int *n, int key)
{
  if (key != 0 && *n < MAX_HELD)
  {
    keys[(*n)++] = key;
  }
}

static bool contains(const int *keys, int n, int key)
{
  for (int i = 0; i < n; i++)
  {
    if (keys[i] == key)
    {
      return true;
    }
  }
  return false;
}

void doom_port_input(void)
{
  touch_points_t p;
  bool used[2] = { false, false };
  const TickType_t now = xTaskGetTickCount();
  touch_get_points(&p);

  /* Fingers already on the joystick or the aiming side keep their role */
  if (s_stick.active && !follow(&s_stick, &p, used))
  {
    s_stick.active = false;
  }
  if (s_aim.active && !follow(&s_aim, &p, used))
  {
    s_aim.active = false;
    if (s_aim_mode == AIM_UNDECIDED)
    {
      s_tap_tics   = TAP_FIRE_TICS;     /* a quick tap */
      s_tap_select = menuactive;
    }
  }

  /* New touches: a button, or the joystick / aiming side */
  const uint32_t was_pressed = s_buttons;
  s_buttons = 0;
  for (int i = 0; i < p.count; i++)
  {
    if (used[i])
    {
      continue;
    }
    const int32_t x = p.x[i], y = p.y[i];
    const int b = button_at(x, y);
    if (b >= 0)
    {
      s_buttons |= 1U << b;
    }
    else if (x >= DOOM_GAME_X && x < DOOM_GAME_X + DOOM_GAME_W / 2 && !s_stick.active)
    {
      s_stick = (finger_t){ true, x, y, x, y };
    }
    else if (x >= DOOM_GAME_X + DOOM_GAME_W / 2 && x < DOOM_GAME_X + DOOM_GAME_W && !s_aim.active)
    {
      s_aim = (finger_t){ true, x, y, x, y };
      s_aim_mode     = AIM_UNDECIDED;
      s_aim_start    = now;
      s_aim_turned_x = x;
    }
  }

  /* Home: when let go */
  if ((was_pressed & (1U << DOOM_BTN_HOME)) != 0U && (s_buttons & (1U << DOOM_BTN_HOME)) == 0U)
  {
    s_wants_out = true;
  }

  /* The keys that should be down now */
  int keys[MAX_HELD];
  int n = 0;
  if (s_buttons & (1U << DOOM_BTN_MENU))   want(keys, &n, key_menu_activate);
  if (s_buttons & (1U << DOOM_BTN_MAP))    want(keys, &n, key_map_toggle);
  /* OK: select, or "yes" to the game's questions (never both: Enter on "Quit"
   * followed by "y" would answer the question it opens) */
  if (s_buttons & (1U << DOOM_BTN_ENTER))  want(keys, &n, messageToPrint ? key_menu_confirm : key_menu_forward);
  if (s_buttons & (1U << DOOM_BTN_USE))    want(keys, &n, key_use);
  if (s_buttons & (1U << DOOM_BTN_WEAPON)) want(keys, &n, key_nextweapon);
  if (s_buttons & (1U << DOOM_BTN_FIRE))   want(keys, &n, key_fire);

  if (s_stick.active)
  {
    const int32_t dx = s_stick.x - s_stick.x0, dy = s_stick.y - s_stick.y0;
    if (dy < -STICK_DEAD) want(keys, &n, menuactive ? key_menu_up : key_up);
    if (dy >  STICK_DEAD) want(keys, &n, menuactive ? key_menu_down : key_down);
    if (dx < -STICK_DEAD) want(keys, &n, menuactive ? key_menu_left : key_strafeleft);
    if (dx >  STICK_DEAD) want(keys, &n, menuactive ? key_menu_right : key_straferight);
  }

  if (s_aim.active)
  {
    const int32_t moved = s_aim.x - s_aim.x0;
    if (s_aim_mode == AIM_UNDECIDED && (moved > AIM_SLOP || moved < -AIM_SLOP))
    {
      s_aim_mode = AIM_TURN;
    }
    else if (s_aim_mode == AIM_UNDECIDED && (now - s_aim_start) >= pdMS_TO_TICKS(AIM_HOLD_MS))
    {
      s_aim_mode = AIM_FIRE;
    }
    if (s_aim_mode == AIM_FIRE)
    {
      want(keys, &n, key_fire);
    }
    /* Turning: the drag since the last tic, as mouse motion */
    if (s_aim_mode != AIM_UNDECIDED)
    {
      const int32_t dx = s_aim.x - s_aim_turned_x;
      s_aim_turned_x = s_aim.x;
      if (dx != 0 && !menuactive)
      {
        event_t ev = { ev_mouse, 0, dx * TURN_SCALE, 0, 0 };
        D_PostEvent(&ev);
      }
    }
  }
  if (s_tap_tics > 0)
  {
    s_tap_tics--;
    want(keys, &n, s_tap_select ? key_menu_forward : key_fire);
  }

  /* Up what is no longer wanted, down what is new */
  int held[MAX_HELD];
  int held_n = 0;
  for (int i = 0; i < s_held_count; i++)
  {
    if (contains(keys, n, s_held[i]))
    {
      held[held_n++] = s_held[i];
    }
    else
    {
      post_key(ev_keyup, s_held[i]);
    }
  }
  for (int i = 0; i < n; i++)
  {
    if (!contains(held, held_n, keys[i]) && held_n < MAX_HELD)
    {
      post_key(ev_keydown, keys[i]);
      held[held_n++] = keys[i];
    }
  }
  memcpy(s_held, held, (size_t)held_n * sizeof(held[0]));
  s_held_count = held_n;
}
