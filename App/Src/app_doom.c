/**
 * @file  app_doom.c
 * @brief Doom app (built with -DWITH_DOOM=ON): Chocolate Doom on the whole
 *        screen, played with touch controls (Doom/port/doom_port.h).
 *
 * Opening the app starts the game, or continues it where it was left; its
 * EXIT button comes back to the home screen. While it runs, LVGL draws
 * nothing (display video mode) and an invisible layer over the app's screen
 * keeps the touches away from LVGL's widgets. The game needs an IWAD (e.g.
 * the shareware DOOM1.WAD) in /doom on the SD card.
 */
#ifdef WITH_DOOM

#include <stdio.h>

#include "display.h"
#include "doom_port.h"
#include "main.h"
#include "player.h"
#include "shell.h"
#include "theme.h"

#define START_DELAY_MS 300      /* after the screen fade: video mode freezes LVGL's drawing */
#define POLL_MS        100

static lv_obj_t   *s_message;
static lv_obj_t   *s_restart;
static lv_obj_t   *s_blocker;   /* over everything while playing: LVGL's widgets get no touches */
static lv_timer_t *s_timer;
static bool        s_playing;
static uint32_t    s_start_at;  /* tick to enter the game at, 0: not waiting */

/* Pictures of the button strips: [left, right][normal, pressed] */
static uint16_t      s_strip_px[2][2][DOOM_STRIP_W * DOOM_SCREEN_H] __attribute__((section(".sdram"), aligned(32)));
static doom_strips_t s_strips;

/* ---- button strips ------------------------------------------------------------ */

static void draw_strip(lv_obj_t *canvas, int strip, bool pressed)
{
  lv_canvas_set_buffer(canvas, s_strip_px[strip][pressed], DOOM_STRIP_W, DOOM_SCREEN_H, LV_COLOR_FORMAT_RGB565);
  lv_canvas_fill_bg(canvas, lv_color_black(), LV_OPA_COVER);

  lv_layer_t layer;
  lv_canvas_init_layer(canvas, &layer);
  const int32_t x0 = (strip == 0) ? 0 : DOOM_SCREEN_W - DOOM_STRIP_W;
  for (int b = 0; b < DOOM_BTN_COUNT; b++)
  {
    const doom_button_rect_t *r = &doom_buttons[b];
    if (r->x < x0 || r->x >= x0 + DOOM_STRIP_W)
    {
      continue;
    }
    const lv_area_t area = { r->x - x0, r->y, r->x - x0 + r->w - 1, r->y + r->h - 1 };

    lv_draw_rect_dsc_t rect;
    lv_draw_rect_dsc_init(&rect);
    rect.radius       = 12;
    rect.bg_color     = pressed ? COLOR_ACCENT : lv_color_hex(0x2A2D33);
    rect.bg_opa       = LV_OPA_COVER;
    rect.border_color = pressed ? COLOR_ACCENT_SOFT : lv_color_hex(0x4A4F57);
    rect.border_width = 2;
    lv_draw_rect(&layer, &rect, &area);

    lv_draw_label_dsc_t label;
    lv_draw_label_dsc_init(&label);
    label.color = lv_color_white();
    label.font  = (b == DOOM_BTN_HOME) ? &lv_font_montserrat_28 : &lv_font_montserrat_20;
    label.align = LV_TEXT_ALIGN_CENTER;
    label.text  = (b == DOOM_BTN_HOME) ? LV_SYMBOL_HOME : r->label;
    const int32_t h = lv_font_get_line_height(label.font);
    const lv_area_t text = { area.x1, area.y1 + (r->h - h) / 2, area.x2, area.y1 + (r->h - h) / 2 + h - 1 };
    lv_draw_label(&layer, &label, &text);
  }
  lv_canvas_finish_layer(canvas, &layer);
}

static void build_strips(lv_obj_t *screen)
{
  lv_obj_t *canvas = lv_canvas_create(screen);
  lv_obj_set_hidden(canvas, true);
  for (int strip = 0; strip < 2; strip++)
  {
    for (int pressed = 0; pressed < 2; pressed++)
    {
      draw_strip(canvas, strip, pressed != 0);
      s_strips[strip][pressed] = s_strip_px[strip][pressed];
    }
  }
  lv_obj_delete(canvas);
  doom_set_strips(&s_strips);
}

/* ---- playing --------------------------------------------------------------------- */

static void show_state(void)
{
  if (doom_state() == DOOM_ENDED)
  {
    const char *why = doom_end_message();
    lv_label_set_text_fmt(s_message, "%s\n\nDoom cannot be started again: restart the board to play.",
                          why != NULL ? why : "Doom has quit.");
    lv_obj_set_hidden(s_restart, false);
  }
  else if (!doom_find_iwad())
  {
    lv_label_set_text(s_message, "Doom needs its game data: copy DOOM1.WAD (the shareware episode), "
                                 "DOOM.WAD or DOOM2.WAD into " DOOM_DIR " on the SD card.");
    lv_obj_set_hidden(s_restart, true);
  }
  else
  {
    lv_label_set_text(s_message, doom_state() == DOOM_PAUSED ? "Continuing..." : "Starting...");
    lv_obj_set_hidden(s_restart, true);
  }
}

static void enter_game(void)
{
  player_status_t st;
  player_get_status(&st);
  player_release_audio();
  lv_obj_set_hidden(s_blocker, false);
  lv_obj_move_foreground(s_blocker);
  display_video_begin();
  s_playing = true;
  doom_resume(st.volume);
}

static void leave_game(void)
{
  if (!s_playing)
  {
    return;
  }
  doom_pause();
  display_video_end();
  lv_obj_set_hidden(s_blocker, true);
  s_playing = false;
}

static void poll(lv_timer_t *timer)
{
  (void)timer;
  if (s_start_at != 0U && lv_tick_elaps(s_start_at) >= START_DELAY_MS)
  {
    s_start_at = 0;
    enter_game();
  }
  if (!s_playing)
  {
    return;
  }
  lv_display_trigger_activity(NULL);   /* no screen timeout while playing */
  if (doom_wants_out())
  {
    leave_game();
    show_state();
    if (doom_state() != DOOM_ENDED || doom_end_message() == NULL)
    {
      shell_go_home();                 /* EXIT, or quit from the game's menu */
    }
  }
}

static void on_restart(lv_event_t *e)
{
  (void)e;
  NVIC_SystemReset();
}

/* ---- app ------------------------------------------------------------------------------ */

static void doom_create(lv_obj_t *screen)
{
  lv_obj_t *c = shell_app_header(screen, "Doom");

  s_message = lv_label_create(c);
  lv_obj_set_style_text_font(s_message, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(s_message, COLOR_TEXT_DIM, 0);
  lv_label_set_long_mode(s_message, LV_LABEL_LONG_MODE_WRAP);
  lv_obj_set_width(s_message, DOOM_SCREEN_W - 2 * 48);
  lv_obj_set_pos(s_message, 48, 48);
  lv_label_set_text(s_message, "");

  s_restart = lv_button_create(c);
  lv_obj_set_size(s_restart, 220, 56);
  lv_obj_set_pos(s_restart, 48, 220);
  lv_obj_set_style_bg_color(s_restart, COLOR_ACCENT, 0);
  lv_obj_add_event_cb(s_restart, on_restart, LV_EVENT_CLICKED, NULL);
  lv_obj_t *label = lv_label_create(s_restart);
  lv_obj_set_style_text_font(label, &lv_font_montserrat_20, 0);
  lv_label_set_text(label, LV_SYMBOL_REFRESH "  Restart board");
  lv_obj_center(label);
  lv_obj_set_hidden(s_restart, true);

  s_blocker = lv_obj_create(screen);
  lv_obj_remove_style_all(s_blocker);
  lv_obj_set_size(s_blocker, DOOM_SCREEN_W, DOOM_SCREEN_H);
  lv_obj_set_clickable(s_blocker, true);
  lv_obj_set_hidden(s_blocker, true);

  build_strips(screen);

  s_timer = lv_timer_create(poll, POLL_MS, NULL);
  lv_timer_pause(s_timer);
}

static void doom_show(void)
{
  show_state();
  if (doom_state() != DOOM_ENDED && doom_find_iwad())
  {
    s_start_at = lv_tick_get();
    if (s_start_at == 0U)
    {
      s_start_at = 1;
    }
  }
  lv_timer_resume(s_timer);
}

static void doom_hide(void)
{
  s_start_at = 0;
  leave_game();
  lv_timer_pause(s_timer);
}

const app_t app_doom = {
  .name   = "Doom",
  .icon   = "D",
  .color  = 0xB3261E,
  .create = doom_create,
  .show   = doom_show,
  .hide   = doom_hide,
};

#endif /* WITH_DOOM */
