/**
 * @file  app_video.c
 * @brief Video app: the AVI files in /Avi, played full screen.
 *
 * While a video plays, LVGL does not draw on the screen: the frames go
 * straight into the display's frame buffers (display video mode). A tap shows
 * the controls over the playing video (top: back, title, stats on/off; bottom:
 * play/pause, seek bar, home) for a few seconds; LVGL renders them off screen
 * (snapshots) and the video player blends them onto every frame, while the
 * touch input works on the LVGL objects as usual. Paused, LVGL draws the
 * screen again: the last frame as background, the same controls over it.
 * At the end the list comes back. Playing a video stops the music.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "display.h"
#include "ff.h"
#include "log.h"
#include "player.h"
#include "shell.h"
#include "theme.h"
#include "video_player.h"
#include "workspace.h"

#define VIDEO_DIR      "/Avi"
#define MAX_VIDEOS     64
#define NAME_LEN       96
#define POLL_MS        100
#define TOP_H          64
#define BAR_H          96
#define CTRL_HIDE_MS   4000     /* controls over a playing video go after this */
#define CTRL_UPDATE_MS 1000     /* their time and seek bar move on this often */
#define START_DELAY_MS 300      /* app_video_play_file: after the screen fade */

/* Layer slots of the video player */
#define LAYER_STATS    0
#define LAYER_TOP      1
#define LAYER_BAR      2

typedef struct
{
  char     name[NAME_LEN];
  uint32_t size_kb;
} entry_t;

static entry_t      s_entries[MAX_VIDEOS];
static uint32_t     s_count;

static lv_obj_t    *s_screen;
static lv_obj_t    *s_list;           /* the file list (below the header) */
static lv_obj_t    *s_empty;
static lv_obj_t    *s_player;         /* full-screen: still frame + controls */
static lv_obj_t    *s_still;
static lv_obj_t    *s_controls;
static lv_obj_t    *s_top;
static lv_obj_t    *s_bar;
static lv_obj_t    *s_play_icon;
static lv_obj_t    *s_slider;
static lv_obj_t    *s_time;
static lv_obj_t    *s_title;
static lv_obj_t    *s_details;
static lv_obj_t    *s_stats_icon;
static lv_timer_t  *s_timer;

static video_info_t s_info;
static bool         s_open;           /* a video is open (the player view is up) */
static const app_t *s_back_to;      /* app to return to when the video closes (NULL: the list) */
static bool         s_playing;        /* frames are being shown (display video mode) */
static bool         s_seeking;        /* the slider is being dragged */
static bool         s_ctrl_shown;     /* the controls are up */
static uint32_t     s_ctrl_touched;   /* tick of the last touch on them */
static uint32_t     s_ctrl_drawn;     /* tick of their last snapshot */
static bool         s_ctrl_dirty;     /* they changed: snapshot again */

/* The frame shown when paused, as the player view's background */
static lv_image_dsc_t s_still_dsc;

/* Controls over the playing video: snapshot (ARGB8888) of each bar, turned
 * into a layer; two sets, the video player blends one while the other is drawn */
static lv_draw_buf_t s_snap;
static video_layer_t s_ctrl_layers[2][2];
static uint32_t      s_ctrl_set;

/* Stats box over the video (fps, decode time, CPU), drawn once a second; two
 * buffers like the controls */
#define STATS_W         330
#define STATS_H         26
#define STATS_PERIOD_MS 1000
static video_layer_t s_stats_layers[2];
static uint32_t      s_stats_set;
static lv_obj_t     *s_stats_canvas;   /* hidden: only draws into s_mem->stats_px */
static bool          s_stats_on = true;
static uint32_t      s_rate_ms, s_rate_shown, s_rate_decode;

/* The pixels of the above, in the workspace while a video is open */
typedef struct
{
  uint16_t still_px[DISPLAY_WIDTH * DISPLAY_HEIGHT] __attribute__((aligned(32)));
  uint8_t  snap_mem[DISPLAY_WIDTH * BAR_H * 4U] __attribute__((aligned(32)));
  uint16_t ctrl_rgb[2][DISPLAY_WIDTH * (TOP_H + BAR_H)] __attribute__((aligned(32)));
  uint8_t  ctrl_alpha[2][DISPLAY_WIDTH * (TOP_H + BAR_H)] __attribute__((aligned(32)));
  uint16_t stats_px[2][STATS_W * STATS_H] __attribute__((aligned(32)));
} video_ui_mem_t;

static video_ui_mem_t *s_mem;

/* ---- helpers --------------------------------------------------------------- */

static lv_obj_t *make_label(lv_obj_t *parent, const lv_font_t *font, lv_color_t color)
{
  lv_obj_t *label = lv_label_create(parent);
  lv_obj_set_style_text_font(label, font, 0);
  lv_obj_set_style_text_color(label, color, 0);
  lv_label_set_text(label, "");
  return label;
}

static lv_obj_t *make_box(lv_obj_t *parent, lv_color_t bg, lv_opa_t opa)
{
  lv_obj_t *box = lv_obj_create(parent);
  lv_obj_set_style_bg_color(box, bg, 0);
  lv_obj_set_style_bg_opa(box, opa, 0);
  lv_obj_set_style_border_width(box, 0, 0);
  lv_obj_set_style_radius(box, 0, 0);
  lv_obj_set_style_pad_all(box, 0, 0);
  lv_obj_set_scrollable(box, false);
  return box;
}

static void format_time(char *buf, size_t len, uint32_t ms)
{
  const uint32_t s = ms / 1000U;
  snprintf(buf, len, "%lu:%02lu", (unsigned long)(s / 60U), (unsigned long)(s % 60U));
}

static bool is_avi(const char *name)
{
  const size_t n = strlen(name);
  return n > 4U && strcasecmp(&name[n - 4U], ".avi") == 0;
}

/* ---- list ------------------------------------------------------------------ */

static int compare_entries(const void *a, const void *b)
{
  return strcmp(((const entry_t *)a)->name, ((const entry_t *)b)->name);
}

static void scan(void)
{
  s_count = 0;
  DIR dir;
  FILINFO fi;
  if (f_opendir(&dir, VIDEO_DIR) != FR_OK)
  {
    return;
  }
  while (s_count < MAX_VIDEOS && f_readdir(&dir, &fi) == FR_OK && fi.fname[0] != '\0')
  {
    if ((fi.fattrib & (AM_DIR | AM_HID | AM_SYS)) == 0U && is_avi(fi.fname) && strlen(fi.fname) < NAME_LEN)
    {
      strcpy(s_entries[s_count].name, fi.fname);
      s_entries[s_count].size_kb = (uint32_t)(fi.fsize / 1024U);
      s_count++;
    }
  }
  f_closedir(&dir);
  qsort(s_entries, s_count, sizeof(entry_t), compare_entries);
}

static void open_video(uint32_t index);

static void on_entry_clicked(lv_event_t *e)
{
  open_video((uint32_t)(uintptr_t)lv_event_get_user_data(e));
}

static void rebuild_list(void)
{
  lv_obj_clean(s_list);
  scan();
  if (s_count == 0U)
  {
    player_status_t st;
    player_get_status(&st);
    lv_label_set_text(s_empty, (st.state == PLAYER_STATE_NO_CARD)
                                 ? "No SD card."
                                 : "No videos: put MJPEG .avi files into " VIDEO_DIR " on the SD card.");
    lv_obj_set_hidden(s_empty, false);
    return;
  }
  lv_obj_set_hidden(s_empty, true);
  for (uint32_t i = 0; i < s_count; i++)
  {
    lv_obj_t *row = make_box(s_list, COLOR_PANEL, LV_OPA_COVER);
    lv_obj_set_size(row, lv_pct(100), 64);
    lv_obj_set_style_radius(row, 12, 0);
    lv_obj_set_style_bg_color(row, COLOR_ACCENT_SOFT, LV_STATE_PRESSED);
    lv_obj_set_clickable(row, true);
    lv_obj_add_event_cb(row, on_entry_clicked, LV_EVENT_CLICKED, (void *)(uintptr_t)i);

    lv_obj_t *icon = make_label(row, &lv_font_montserrat_28, COLOR_ACCENT);
    lv_label_set_text(icon, LV_SYMBOL_VIDEO);
    lv_obj_align(icon, LV_ALIGN_LEFT_MID, 20, 0);

    lv_obj_t *size = make_label(row, &lv_font_montserrat_20, COLOR_TEXT_DIM);
    const uint32_t kb = s_entries[i].size_kb;
    if (kb >= 1024U)
    {
      lv_label_set_text_fmt(size, "%lu.%lu MB", (unsigned long)(kb / 1024U), (unsigned long)(kb % 1024U * 10U / 1024U));
    }
    else
    {
      lv_label_set_text_fmt(size, "%lu KB", (unsigned long)kb);
    }
    lv_obj_align(size, LV_ALIGN_RIGHT_MID, -20, 0);

    lv_obj_t *name = make_label(row, shell_font_text(), COLOR_TEXT);
    lv_label_set_long_mode(name, LV_LABEL_LONG_MODE_DOTS);
    lv_obj_set_size(name, 560, lv_font_get_line_height(shell_font_text()));
    lv_label_set_text(name, s_entries[i].name);
    lv_obj_align(name, LV_ALIGN_LEFT_MID, 72, 0);
  }
}

/* ---- layers over the video -------------------------------------------------- */

/* ARGB8888 snapshot -> premultiplied RGB565 + alpha */
static void to_layer(const lv_draw_buf_t *src, uint16_t *rgb, uint8_t *alpha)
{
  const uint32_t w = src->header.w;
  const uint32_t h = src->header.h;
  for (uint32_t y = 0; y < h; y++)
  {
    const uint8_t *p = src->data + y * src->header.stride;
    for (uint32_t x = 0; x < w; x++, p += 4)
    {
      const uint32_t a = p[3];
      const uint32_t r = p[2] * a / 255U;
      const uint32_t g = p[1] * a / 255U;
      const uint32_t b = p[0] * a / 255U;
      *rgb++   = (uint16_t)(((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3));
      *alpha++ = (uint8_t)a;
    }
  }
}

static bool snapshot(lv_obj_t *obj, uint32_t h, uint16_t *rgb, uint8_t *alpha)
{
  if (lv_draw_buf_init(&s_snap, DISPLAY_WIDTH, h, LV_COLOR_FORMAT_ARGB8888, DISPLAY_WIDTH * 4U, s_mem->snap_mem,
                       sizeof(s_mem->snap_mem)) != LV_RESULT_OK ||
      lv_snapshot_take_to_draw_buf(obj, LV_COLOR_FORMAT_ARGB8888, &s_snap) != LV_RESULT_OK ||
      s_snap.header.w != DISPLAY_WIDTH || s_snap.header.h != h)
  {
    return false;
  }
  to_layer(&s_snap, rgb, alpha);
  return true;
}

/* Render the controls (as they are now) over the playing video */
static void draw_controls_layers(void)
{
  const uint32_t k = s_ctrl_set ^ 1U;
  lv_obj_update_layout(s_player);   /* LVGL does not refresh the screen meanwhile */
  if (!snapshot(s_top, TOP_H, s_mem->ctrl_rgb[k], s_mem->ctrl_alpha[k]) ||
      !snapshot(s_bar, BAR_H, &s_mem->ctrl_rgb[k][DISPLAY_WIDTH * TOP_H], &s_mem->ctrl_alpha[k][DISPLAY_WIDTH * TOP_H]))
  {
    LOG_WARN("video: controls snapshot failed");
    return;
  }
  s_ctrl_layers[k][0] = (video_layer_t){ s_mem->ctrl_rgb[k], s_mem->ctrl_alpha[k], 0, 0, DISPLAY_WIDTH, TOP_H };
  s_ctrl_layers[k][1] = (video_layer_t){ &s_mem->ctrl_rgb[k][DISPLAY_WIDTH * TOP_H], &s_mem->ctrl_alpha[k][DISPLAY_WIDTH * TOP_H],
                                         0, DISPLAY_HEIGHT - BAR_H, DISPLAY_WIDTH, BAR_H };
  video_set_layer(LAYER_TOP, &s_ctrl_layers[k][0]);
  video_set_layer(LAYER_BAR, &s_ctrl_layers[k][1]);
  s_ctrl_set   = k;
  s_ctrl_drawn = lv_tick_get();
  s_ctrl_dirty = false;
}

static void remove_controls_layers(void)
{
  video_set_layer(LAYER_TOP, NULL);
  video_set_layer(LAYER_BAR, NULL);
}

static void draw_stats(const char *text)
{
  const uint32_t k = s_stats_set ^ 1U;
  lv_canvas_set_buffer(s_stats_canvas, s_mem->stats_px[k], STATS_W, STATS_H, LV_COLOR_FORMAT_RGB565);
  lv_canvas_fill_bg(s_stats_canvas, lv_color_black(), LV_OPA_COVER);

  lv_layer_t layer;
  lv_canvas_init_layer(s_stats_canvas, &layer);
  lv_draw_label_dsc_t dsc;
  lv_draw_label_dsc_init(&dsc);
  dsc.color = lv_color_white();
  dsc.font  = &lv_font_montserrat_14;
  dsc.text  = text;
  const lv_area_t area = { 8, 5, STATS_W - 1, STATS_H - 1 };
  lv_draw_label(&layer, &dsc, &area);
  lv_canvas_finish_layer(s_stats_canvas, &layer);

  /* Below the top bar, so that the controls never cover it */
  s_stats_layers[k] = (video_layer_t){ s_mem->stats_px[k], NULL, 12, TOP_H + 12, STATS_W, STATS_H };
  video_set_layer(LAYER_STATS, &s_stats_layers[k]);
  s_stats_set = k;
}

/* Once a second while playing: the rates over the last second */
static void update_stats(bool restart)
{
  uint32_t shown, dropped, decode_ms;
  video_stats(&shown, &dropped, &decode_ms);
  const uint32_t now = lv_tick_get();
  if (!restart && s_stats_on)
  {
    const uint32_t elapsed = now - s_rate_ms;
    const uint32_t frames  = shown - s_rate_shown;
    const uint32_t fps_x10 = (elapsed != 0U) ? frames * 10000U / elapsed : 0U;
    char text[80];
    snprintf(text, sizeof(text), "%lu.%lu fps   decode %lu ms   CPU %lu%%   dropped %lu",
             (unsigned long)(fps_x10 / 10U), (unsigned long)(fps_x10 % 10U),
             (unsigned long)((frames != 0U) ? (decode_ms - s_rate_decode) / frames : 0U),
             (unsigned long)shell_cpu_load(), (unsigned long)dropped);
    draw_stats(text);
  }
  s_rate_ms     = now;
  s_rate_shown  = shown;
  s_rate_decode = decode_ms;
}

/* ---- playback ---------------------------------------------------------------- */

static void log_stats(const char *what)
{
  uint32_t shown, dropped, decode_ms;
  video_stats(&shown, &dropped, &decode_ms);
  LOG_INFO("video: %s at %lu ms, %lu frames shown, %lu dropped, %lu ms per frame", what,
           (unsigned long)video_position_ms(), (unsigned long)shown, (unsigned long)dropped,
           (unsigned long)((shown != 0U) ? decode_ms / shown : 0U));
}

static void update_time(void)
{
  char pos[16], dur[16];
  const uint32_t at = s_seeking ? (uint32_t)lv_slider_get_value(s_slider) * 1000U : video_position_ms();
  format_time(pos, sizeof(pos), at);
  format_time(dur, sizeof(dur), s_info.duration_ms);
  lv_label_set_text_fmt(s_time, "%s / %s", pos, dur);
  if (!s_seeking)
  {
    lv_slider_set_value(s_slider, (int32_t)(at / 1000U), LV_ANIM_OFF);
  }
}

static void show_controls(void)
{
  s_ctrl_shown   = true;
  s_ctrl_touched = lv_tick_get();
  update_time();
  lv_obj_set_hidden(s_controls, false);
  if (s_playing)
  {
    draw_controls_layers();
  }
}

static void hide_controls(void)
{
  s_ctrl_shown = false;
  lv_obj_set_hidden(s_controls, true);
  if (s_playing)
  {
    remove_controls_layers();
  }
}

/* Frames to the screen, LVGL off it */
static void start_playing(void)
{
  lv_label_set_text(s_play_icon, LV_SYMBOL_PAUSE);
  update_stats(true);
  display_video_begin();             /* the screen keeps the still until the first frame */
  lv_obj_set_hidden(s_still, true);
  s_playing = true;
  if (s_ctrl_shown)
  {
    s_ctrl_touched = lv_tick_get();
    draw_controls_layers();
  }
  video_play();
}

/* LVGL back on the screen, over the last frame */
static void stop_playing(void)
{
  if (!s_playing)
  {
    return;
  }
  video_pause();
  log_stats("paused");
  video_copy_frame(s_mem->still_px);
  video_set_layer(LAYER_STATS, NULL);
  remove_controls_layers();
  lv_image_set_src(s_still, &s_still_dsc);
  lv_obj_set_hidden(s_still, false);
  lv_label_set_text(s_play_icon, LV_SYMBOL_PLAY);
  s_playing = false;
  display_video_end();
}

static void close_video(void)
{
  stop_playing();
  if (s_open)
  {
    video_close();
    s_open = false;
    hide_controls();
    lv_obj_set_hidden(s_player, true);
    lv_image_set_src(s_still, NULL);
    s_mem = NULL;
    workspace_give(WORKSPACE_VIDEO);
  }
}

/* Open a file and show the player view over the list; false if it cannot be played */
static bool open_file(const char *path, const char *name)
{
  /* All the big buffers come from the workspace, for as long as the video is open */
  s_mem = workspace_take(WORKSPACE_VIDEO) ? workspace_alloc(WORKSPACE_VIDEO, sizeof(*s_mem)) : NULL;
  if (s_mem == NULL || !video_open(path, &s_info))
  {
    s_mem = NULL;
    workspace_give(WORKSPACE_VIDEO);
    return false;
  }
  s_still_dsc.data = (const uint8_t *)s_mem->still_px;
  player_release_audio();
  s_open = true;

  lv_label_set_text(s_title, name);
  lv_label_set_text_fmt(s_details, "%lu x %lu, %lu.%02lu fps, %s", (unsigned long)s_info.width,
                        (unsigned long)s_info.height, (unsigned long)(s_info.fps_x100 / 100U),
                        (unsigned long)(s_info.fps_x100 % 100U), s_info.audio);
  const int32_t seconds = (int32_t)(s_info.duration_ms / 1000U);
  lv_slider_set_range(s_slider, 0, seconds > 0 ? seconds : 1);
  lv_slider_set_value(s_slider, 0, LV_ANIM_OFF);

  /* The player view covers the screen (black until the first frame) */
  lv_obj_set_hidden(s_still, true);
  lv_obj_set_hidden(s_player, false);
  lv_obj_move_foreground(s_player);
  return true;
}

static void open_video(uint32_t index)
{
  char path[sizeof(VIDEO_DIR) + NAME_LEN + 1];
  snprintf(path, sizeof(path), VIDEO_DIR "/%s", s_entries[index].name);
  if (open_file(path, s_entries[index].name))
  {
    start_playing();
  }
}

/* Play once the screen has faded in: video mode freezes LVGL's drawing */
static void on_start_timer(lv_timer_t *timer)
{
  (void)timer;
  if (s_open && !s_playing && lv_screen_active() == s_screen)
  {
    start_playing();
  }
}

bool app_video_play_file(const char *path, const app_t *back_to)
{
  const char *slash = strrchr(path, '/');
  if (!open_file(path, slash != NULL ? slash + 1 : path))
  {
    return false;
  }
  shell_open(&app_video);
  s_back_to = back_to;
  lv_timer_t *timer = lv_timer_create(on_start_timer, START_DELAY_MS, NULL);
  lv_timer_set_repeat_count(timer, 1);
  return true;
}

/* A tap on the picture: controls on / off (paused, they stay) */
static void on_player_clicked(lv_event_t *e)
{
  (void)e;
  if (!s_ctrl_shown)
  {
    show_controls();
  }
  else if (s_playing)
  {
    hide_controls();
  }
}

/* Any touch on the controls keeps them up, and may change how they look */
static void on_ctrl_touched(lv_event_t *e)
{
  (void)e;
  s_ctrl_touched = lv_tick_get();
  s_ctrl_dirty   = true;
}

static void on_play_clicked(lv_event_t *e)
{
  (void)e;
  if (s_playing)
  {
    stop_playing();
  }
  else
  {
    start_playing();
  }
}

/* Close the video: back to the list, or to the app that opened it */
static void leave_video(void)
{
  const app_t *back_to = s_back_to;
  s_back_to = NULL;
  close_video();
  if (back_to != NULL)
  {
    shell_open(back_to);
  }
}

static void on_back_clicked(lv_event_t *e)
{
  (void)e;
  leave_video();
}

static void on_stats_clicked(lv_event_t *e)
{
  (void)e;
  s_stats_on = !s_stats_on;
  lv_label_set_text(s_stats_icon, s_stats_on ? LV_SYMBOL_EYE_OPEN : LV_SYMBOL_EYE_CLOSE);
  if (!s_stats_on)
  {
    video_set_layer(LAYER_STATS, NULL);
  }
}

static void on_slider_event(lv_event_t *e)
{
  const lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_PRESSED)
  {
    s_seeking = true;
  }
  else if (code == LV_EVENT_VALUE_CHANGED && s_seeking)
  {
    update_time();
  }
  else if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST)
  {
    s_seeking = false;
    video_seek((uint32_t)lv_slider_get_value(s_slider) * 1000U);
    update_time();
  }
}

static void poll(lv_timer_t *timer)
{
  (void)timer;
  if (!s_playing)
  {
    return;
  }
  lv_display_trigger_activity(NULL);   /* no screen timeout while watching */
  if (video_state() == VIDEO_ENDED)
  {
    log_stats("ended");
    leave_video();
    return;
  }
  if (s_ctrl_shown)
  {
    if (!s_seeking && lv_tick_elaps(s_ctrl_touched) >= CTRL_HIDE_MS)
    {
      hide_controls();
    }
    else if (s_ctrl_dirty || lv_tick_elaps(s_ctrl_drawn) >= CTRL_UPDATE_MS)
    {
      update_time();
      draw_controls_layers();
    }
  }
  if (lv_tick_elaps(s_rate_ms) >= STATS_PERIOD_MS)
  {
    update_stats(false);
  }
}

/* ---- app ------------------------------------------------------------------ */

static lv_obj_t *make_icon_button(lv_obj_t *parent, const char *symbol, int32_t size, lv_color_t bg, lv_color_t fg)
{
  lv_obj_t *btn = lv_button_create(parent);
  lv_obj_set_size(btn, size, size);
  lv_obj_set_style_radius(btn, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_bg_color(btn, bg, 0);
  lv_obj_set_style_shadow_width(btn, 0, 0);
  lv_obj_set_ext_click_area(btn, 8);
  lv_obj_add_event_cb(btn, on_ctrl_touched, LV_EVENT_PRESSED, NULL);
  lv_obj_add_event_cb(btn, on_ctrl_touched, LV_EVENT_RELEASED, NULL);
  lv_obj_t *icon = make_label(btn, &lv_font_montserrat_28, fg);
  lv_label_set_text(icon, symbol);
  lv_obj_center(icon);
  return btn;
}

static lv_obj_t *make_flat_button(lv_obj_t *parent, const char *symbol)
{
  lv_obj_t *btn = make_icon_button(parent, symbol, 44, lv_color_white(), lv_color_white());
  lv_obj_set_style_bg_opa(btn, LV_OPA_TRANSP, 0);
  lv_obj_set_style_bg_opa(btn, LV_OPA_30, LV_STATE_PRESSED);
  return btn;
}

static void build_player(lv_obj_t *screen)
{
  s_player = make_box(screen, lv_color_black(), LV_OPA_COVER);
  lv_obj_set_size(s_player, DISPLAY_WIDTH, DISPLAY_HEIGHT);
  lv_obj_set_clickable(s_player, true);
  lv_obj_set_hidden(s_player, true);
  lv_obj_add_event_cb(s_player, on_player_clicked, LV_EVENT_CLICKED, NULL);

  s_stats_canvas = lv_canvas_create(s_player);
  lv_obj_set_hidden(s_stats_canvas, true);

  s_still_dsc.header.magic  = LV_IMAGE_HEADER_MAGIC;
  s_still_dsc.header.cf     = LV_COLOR_FORMAT_RGB565;
  s_still_dsc.header.w      = DISPLAY_WIDTH;
  s_still_dsc.header.h      = DISPLAY_HEIGHT;
  s_still_dsc.header.stride = DISPLAY_WIDTH * 2U;
  s_still_dsc.data_size     = sizeof(s_mem->still_px);
  s_still_dsc.data          = NULL;   /* set when a video is opened (workspace) */
  s_still = lv_image_create(s_player);
  lv_obj_set_pos(s_still, 0, 0);
  lv_obj_set_hidden(s_still, true);

  /* Controls: translucent bars at the top and the bottom */
  s_controls = make_box(s_player, lv_color_black(), LV_OPA_TRANSP);
  lv_obj_set_size(s_controls, DISPLAY_WIDTH, DISPLAY_HEIGHT);
  lv_obj_set_clickable(s_controls, false);
  lv_obj_set_hidden(s_controls, true);

  s_top = make_box(s_controls, lv_color_black(), LV_OPA_60);
  lv_obj_set_size(s_top, DISPLAY_WIDTH, TOP_H);
  lv_obj_add_event_cb(s_top, on_ctrl_touched, LV_EVENT_PRESSED, NULL);
  lv_obj_t *back = make_flat_button(s_top, LV_SYMBOL_LEFT);
  lv_obj_set_pos(back, 18, 10);
  lv_obj_add_event_cb(back, on_back_clicked, LV_EVENT_CLICKED, NULL);
  s_title = make_label(s_top, shell_font_text(), lv_color_white());
  lv_label_set_long_mode(s_title, LV_LABEL_LONG_MODE_DOTS);
  lv_obj_set_size(s_title, 620, lv_font_get_line_height(shell_font_text()));
  lv_obj_set_pos(s_title, 76, 18);
  lv_obj_t *stats = make_flat_button(s_top, LV_SYMBOL_EYE_OPEN);
  lv_obj_set_pos(stats, DISPLAY_WIDTH - 18 - 44, 10);
  lv_obj_add_event_cb(stats, on_stats_clicked, LV_EVENT_CLICKED, NULL);
  s_stats_icon = lv_obj_get_child(stats, 0);

  s_bar = make_box(s_controls, lv_color_black(), LV_OPA_60);
  lv_obj_set_size(s_bar, DISPLAY_WIDTH, BAR_H);
  lv_obj_set_pos(s_bar, 0, DISPLAY_HEIGHT - BAR_H);
  lv_obj_add_event_cb(s_bar, on_ctrl_touched, LV_EVENT_PRESSED, NULL);

  lv_obj_t *play = make_icon_button(s_bar, LV_SYMBOL_PAUSE, 64, COLOR_ACCENT, COLOR_ON_ACCENT);
  lv_obj_set_style_bg_color(play, COLOR_ACCENT_DARK, LV_STATE_PRESSED);
  lv_obj_align(play, LV_ALIGN_LEFT_MID, 24, 0);
  lv_obj_add_event_cb(play, on_play_clicked, LV_EVENT_CLICKED, NULL);
  s_play_icon = lv_obj_get_child(play, 0);

  s_slider = lv_slider_create(s_bar);
  lv_obj_set_size(s_slider, 560, 8);
  lv_obj_set_pos(s_slider, 120, 28);
  lv_obj_set_ext_click_area(s_slider, 20);
  lv_obj_set_style_bg_color(s_slider, lv_color_hex(0x5F6875), LV_PART_MAIN);
  lv_obj_set_style_bg_color(s_slider, COLOR_ACCENT, LV_PART_INDICATOR);
  lv_obj_set_style_bg_color(s_slider, lv_color_white(), LV_PART_KNOB);
  lv_obj_add_event_cb(s_slider, on_slider_event, LV_EVENT_ALL, NULL);
  lv_obj_add_event_cb(s_slider, on_ctrl_touched, LV_EVENT_PRESSING, NULL);

  s_time = make_label(s_bar, &lv_font_montserrat_20, lv_color_white());
  lv_obj_set_pos(s_time, 120, 52);
  s_details = make_label(s_bar, &lv_font_montserrat_14, lv_color_hex(0xC8CDD4));
  lv_obj_align(s_details, LV_ALIGN_TOP_RIGHT, -96, 56);
  lv_obj_set_style_text_align(s_details, LV_TEXT_ALIGN_RIGHT, 0);

  lv_obj_t *home = shell_home_button(s_bar);
  lv_obj_set_style_text_color(lv_obj_get_child(home, 0), lv_color_white(), 0);
  lv_obj_set_style_bg_color(home, lv_color_white(), LV_STATE_PRESSED);
  lv_obj_set_style_bg_opa(home, LV_OPA_30, LV_STATE_PRESSED);
  lv_obj_align(home, LV_ALIGN_RIGHT_MID, -24, 0);
}

static void video_create(lv_obj_t *screen)
{
  s_screen = screen;
  lv_obj_t *content = shell_app_header(screen, "Video");

  s_list = make_box(content, COLOR_BG, LV_OPA_COVER);
  lv_obj_set_size(s_list, DISPLAY_WIDTH, DISPLAY_HEIGHT - 66);
  lv_obj_set_pos(s_list, 0, 1);
  lv_obj_set_style_pad_hor(s_list, 32, 0);
  lv_obj_set_style_pad_ver(s_list, 16, 0);
  lv_obj_set_style_pad_row(s_list, 10, 0);
  lv_obj_set_flex_flow(s_list, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_scrollable(s_list, true);
  lv_obj_set_scroll_dir(s_list, LV_DIR_VER);

  s_empty = make_label(content, &lv_font_montserrat_20, COLOR_TEXT_DIM);
  lv_obj_set_width(s_empty, DISPLAY_WIDTH - 64);
  lv_label_set_long_mode(s_empty, LV_LABEL_LONG_MODE_WRAP);
  lv_obj_set_pos(s_empty, 32, 32);
  lv_obj_set_hidden(s_empty, true);

  build_player(screen);
  s_timer = lv_timer_create(poll, POLL_MS, NULL);
  lv_timer_pause(s_timer);
}

static void video_show(void)
{
  rebuild_list();
  lv_timer_resume(s_timer);
}

static void video_hide(void)
{
  s_back_to = NULL;
  close_video();
  lv_timer_pause(s_timer);
}

const app_t app_video = {
  .name   = "Video",
  .icon   = LV_SYMBOL_VIDEO,
  .color  = 0xE8590C,
  .create = video_create,
  .show   = video_show,
  .hide   = video_hide,
};
