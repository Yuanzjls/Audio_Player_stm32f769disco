/**
 * @file  ui.c
 * @brief Player screen: title, seek bar, transport buttons and volume.
 *
 * The UI never touches the audio hardware: it sends commands through the
 * player API and refreshes itself from player_get_status() on a timer.
 * Song titles are drawn with a TrueType font loaded from the SD card
 * (font.ttf / font.otf / font.ttc in the root) so that CJK names render correctly;
 * without it the built-in Latin font is used.
 */
#include "ui.h"

#include <stdio.h>
#include <string.h>

#include "lvgl.h"
#include "log.h"
#include "player.h"

#define REFRESH_PERIOD_MS   100
#define TITLE_FONT_SIZE     36
#define SLIDER_HEIGHT       14
#define SEEK_TOUCH_MARGIN   36      /* px around the bar that still grab it */
#define VOLUME_TOUCH_MARGIN 28      /* smaller: the play button is just above */

#define COLOR_BG        lv_color_hex(0xF4F6F9)
#define COLOR_SURFACE   lv_color_hex(0xE2E6EC)
#define COLOR_ACCENT    lv_color_hex(0x1A73E8)
#define COLOR_TEXT      lv_color_hex(0x1F2328)
#define COLOR_TEXT_DIM  lv_color_hex(0x5F6875)
#define COLOR_WARNING   lv_color_hex(0xB45309)
#define COLOR_ON_ACCENT lv_color_hex(0xFFFFFF)

static const char *const k_font_paths[] = { "S:/font.ttf", "S:/font.otf", "S:/font.ttc" };

typedef struct
{
  lv_obj_t *title;
  lv_obj_t *track_info;
  lv_obj_t *format;
  lv_obj_t *message;
  lv_obj_t *seek;
  lv_obj_t *time_pos;
  lv_obj_t *time_total;
  lv_obj_t *btn_play;
  lv_obj_t *btn_play_icon;
  lv_obj_t *btn_next;
  lv_obj_t *btn_repeat;
  lv_obj_t *btn_repeat_icon;
  lv_obj_t *volume;
  lv_obj_t *volume_icon;
  lv_obj_t *volume_value;
} ui_widgets_t;

static ui_widgets_t s_ui;
static player_status_t s_status;      /* last status shown */
static uint32_t s_shown_serial = UINT32_MAX;
static bool     s_first_refresh = true;
static int32_t  s_last_volume_sent = -1;
static lv_font_t *s_title_font;
static bool     s_font_tried;

/* ---- helpers --------------------------------------------------------------- */

static void format_time(char *buf, size_t len, uint32_t seconds)
{
  snprintf(buf, len, "%02lu:%02lu", (unsigned long)(seconds / 60U), (unsigned long)(seconds % 60U));
}

static void set_label_time(lv_obj_t *label, uint32_t seconds)
{
  char buf[12];
  format_time(buf, sizeof(buf), seconds);
  lv_label_set_text(label, buf);
}

static lv_obj_t *make_label(lv_obj_t *parent, const lv_font_t *font, lv_color_t color)
{
  lv_obj_t *label = lv_label_create(parent);
  lv_obj_set_style_text_font(label, font, 0);
  lv_obj_set_style_text_color(label, color, 0);
  return label;
}

static lv_obj_t *make_round_button(lv_obj_t *parent, int32_t size, lv_color_t bg, lv_color_t fg,
                                   const char *symbol, const lv_font_t *font, lv_obj_t **icon_out)
{
  lv_obj_t *btn = lv_button_create(parent);
  lv_obj_set_size(btn, size, size);
  lv_obj_set_style_radius(btn, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_bg_color(btn, bg, 0);
  lv_obj_set_style_shadow_width(btn, 0, 0);

  lv_obj_t *icon = make_label(btn, font, fg);
  lv_label_set_text(icon, symbol);
  lv_obj_center(icon);
  if (icon_out != NULL)
  {
    *icon_out = icon;
  }
  return btn;
}

/* A finger is much less precise than LVGL's default slider (an ~8 px bar with an
 * 8 px touch margin): use a thicker bar, a bigger knob and a wide touch margin.
 * Tapping anywhere in the margin jumps the knob there. */
static void style_slider(lv_obj_t *slider, int32_t touch_margin)
{
  lv_obj_set_height(slider, SLIDER_HEIGHT);
  lv_obj_set_ext_click_area(slider, touch_margin);
  lv_obj_set_style_bg_color(slider, COLOR_SURFACE, LV_PART_MAIN);
  lv_obj_set_style_bg_opa(slider, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_bg_color(slider, COLOR_ACCENT, LV_PART_INDICATOR);
  lv_obj_set_style_bg_color(slider, COLOR_ON_ACCENT, LV_PART_KNOB);
  lv_obj_set_style_border_color(slider, COLOR_ACCENT, LV_PART_KNOB);
  lv_obj_set_style_border_width(slider, 3, LV_PART_KNOB);
  lv_obj_set_style_pad_all(slider, 8, LV_PART_KNOB);
}

/* ---- event handlers -------------------------------------------------------- */

static void on_play_clicked(lv_event_t *e)
{
  (void)e;
  player_toggle_play();
}

static void on_next_clicked(lv_event_t *e)
{
  (void)e;
  player_next();
}

static void on_repeat_clicked(lv_event_t *e)
{
  (void)e;
  player_set_repeat(s_status.repeat == PLAYER_REPEAT_ONE ? PLAYER_REPEAT_OFF : PLAYER_REPEAT_ONE);
}

static void on_seek_event(lv_event_t *e)
{
  lv_obj_t *slider = lv_event_get_target_obj(e);
  const lv_event_code_t code = lv_event_get_code(e);

  if (code == LV_EVENT_VALUE_CHANGED)
  {
    /* Preview the target position while dragging */
    set_label_time(s_ui.time_pos, (uint32_t)lv_slider_get_value(slider));
  }
  else if (code == LV_EVENT_RELEASED)
  {
    player_seek((uint32_t)lv_slider_get_value(slider));
  }
}

static void on_volume_event(lv_event_t *e)
{
  lv_obj_t *slider = lv_event_get_target_obj(e);
  const int32_t value = lv_slider_get_value(slider);

  lv_label_set_text_fmt(s_ui.volume_value, "%" LV_PRId32, value);
  /* Send only real changes so a drag does not flood the player's queue */
  if (value != s_last_volume_sent)
  {
    s_last_volume_sent = value;
    player_set_volume((uint8_t)value);
  }
}

/* ---- refresh --------------------------------------------------------------- */

static bool file_exists(const char *path)
{
  lv_fs_file_t file;
  if (lv_fs_open(&file, path, LV_FS_MODE_RD) != LV_FS_RES_OK)
  {
    return false;
  }
  lv_fs_close(&file);
  return true;
}

/* Called once the SD card is mounted; tries a single time. */
static void try_load_title_font(void)
{
  if (s_font_tried)
  {
    return;
  }
  s_font_tried = true;

  for (size_t i = 0; i < sizeof(k_font_paths) / sizeof(k_font_paths[0]); i++)
  {
    if (!file_exists(k_font_paths[i]))
    {
      continue;
    }
    s_title_font = lv_tiny_ttf_create_file(k_font_paths[i], TITLE_FONT_SIZE);
    if (s_title_font == NULL)
    {
      LOG_WARN("ui: %s is not a usable TrueType/OpenType font", k_font_paths[i]);
      continue;
    }
    /* Glyphs missing from the TTF fall back to the built-in font */
    s_title_font->fallback = &lv_font_montserrat_28;
    lv_obj_set_style_text_font(s_ui.title, s_title_font, 0);
    LOG_INFO("ui: title font %s loaded", k_font_paths[i]);
    return;
  }
  LOG_WARN("ui: no font.ttf/font.otf/font.ttc on the SD card, non-Latin titles will not render");
}

static void refresh_track(const player_status_t *st)
{
  char buf[48];

  lv_label_set_text(s_ui.title, st->title);
  lv_label_set_text_fmt(s_ui.track_info, "Track %u of %u",
                        (unsigned)st->track_index + 1U, (unsigned)st->track_count);
  snprintf(buf, sizeof(buf), "%lu.%lu kHz  %s", (unsigned long)(st->sample_rate / 1000U),
           (unsigned long)((st->sample_rate % 1000U) / 100U), st->channels == 1U ? "mono" : "stereo");
  lv_label_set_text(s_ui.format, buf);

  lv_slider_set_range(s_ui.seek, 0, (int32_t)(st->duration_s > 0U ? st->duration_s : 1U));
  set_label_time(s_ui.time_total, st->duration_s);
}

static void refresh_cb(lv_timer_t *timer)
{
  (void)timer;
  player_status_t st;
  player_get_status(&st);

  const bool has_card   = (st.state != PLAYER_STATE_NO_CARD);
  const bool has_tracks = has_card && (st.state != PLAYER_STATE_NO_TRACKS);

  if (has_card)
  {
    try_load_title_font();
  }

  if (has_tracks && st.track_serial != s_shown_serial)
  {
    s_shown_serial = st.track_serial;
    refresh_track(&st);
  }

  /* User hint / error message */
  if (s_first_refresh || strcmp(st.message, s_status.message) != 0 || st.state != s_status.state)
  {
    s_first_refresh = false;
    if (st.message[0] != '\0')
    {
      lv_label_set_text_fmt(s_ui.message, LV_SYMBOL_WARNING "  %s", st.message);
      lv_obj_set_hidden(s_ui.message, false);
    }
    else
    {
      lv_obj_set_hidden(s_ui.message, true);
    }
    if (!has_tracks)
    {
      lv_label_set_text(s_ui.title, "");
      lv_label_set_text(s_ui.track_info, "");
      lv_label_set_text(s_ui.format, "");
      s_shown_serial = UINT32_MAX;
    }
  }

  /* Transport controls */
  lv_label_set_text(s_ui.btn_play_icon, st.state == PLAYER_STATE_PLAYING ? LV_SYMBOL_PAUSE : LV_SYMBOL_PLAY);
  if (has_tracks)
  {
    lv_obj_remove_state(s_ui.btn_play, LV_STATE_DISABLED);
    lv_obj_remove_state(s_ui.btn_next, LV_STATE_DISABLED);
    lv_obj_remove_state(s_ui.seek, LV_STATE_DISABLED);
  }
  else
  {
    lv_obj_add_state(s_ui.btn_play, LV_STATE_DISABLED);
    lv_obj_add_state(s_ui.btn_next, LV_STATE_DISABLED);
    lv_obj_add_state(s_ui.seek, LV_STATE_DISABLED);
  }

  const bool repeat_one = (st.repeat == PLAYER_REPEAT_ONE);
  lv_obj_set_style_bg_color(s_ui.btn_repeat, repeat_one ? COLOR_ACCENT : COLOR_SURFACE, 0);
  lv_obj_set_style_text_color(s_ui.btn_repeat_icon, repeat_one ? COLOR_ON_ACCENT : COLOR_TEXT, 0);

  /* Progress, unless the user is dragging the seek bar */
  if (!lv_slider_is_dragged(s_ui.seek))
  {
    lv_slider_set_value(s_ui.seek, (int32_t)st.position_s, LV_ANIM_OFF);
    set_label_time(s_ui.time_pos, st.position_s);
  }

  s_status = st;
}

/* ---- construction ---------------------------------------------------------- */

static void build_header(lv_obj_t *scr)
{
  lv_obj_t *app_name = make_label(scr, &lv_font_montserrat_20, COLOR_TEXT_DIM);
  lv_label_set_text(app_name, LV_SYMBOL_AUDIO "  Audio Player");
  lv_obj_align(app_name, LV_ALIGN_TOP_LEFT, 24, 20);

  s_ui.format = make_label(scr, &lv_font_montserrat_20, COLOR_TEXT_DIM);
  lv_label_set_text(s_ui.format, "");
  lv_obj_align(s_ui.format, LV_ALIGN_TOP_RIGHT, -24, 20);
}

static void build_track_info(lv_obj_t *scr)
{
  s_ui.title = make_label(scr, &lv_font_montserrat_28, COLOR_TEXT);
  lv_obj_set_width(s_ui.title, 720);
  lv_obj_set_style_text_align(s_ui.title, LV_TEXT_ALIGN_CENTER, 0);
  lv_label_set_long_mode(s_ui.title, LV_LABEL_LONG_MODE_SCROLL_CIRCULAR);
  lv_label_set_text(s_ui.title, "");
  lv_obj_align(s_ui.title, LV_ALIGN_TOP_MID, 0, 100);

  s_ui.track_info = make_label(scr, &lv_font_montserrat_20, COLOR_TEXT_DIM);
  lv_label_set_text(s_ui.track_info, "");
  lv_obj_align(s_ui.track_info, LV_ALIGN_TOP_MID, 0, 160);

  s_ui.message = make_label(scr, &lv_font_montserrat_28, COLOR_WARNING);
  lv_label_set_text(s_ui.message, "");
  lv_obj_align(s_ui.message, LV_ALIGN_TOP_MID, 0, 110);
  lv_obj_set_hidden(s_ui.message, true);
}

static void build_seek_bar(lv_obj_t *scr)
{
  s_ui.seek = lv_slider_create(scr);
  lv_obj_set_width(s_ui.seek, 640);
  lv_obj_align(s_ui.seek, LV_ALIGN_TOP_MID, 0, 225);
  lv_slider_set_range(s_ui.seek, 0, 1);
  style_slider(s_ui.seek, SEEK_TOUCH_MARGIN);
  lv_obj_add_event_cb(s_ui.seek, on_seek_event, LV_EVENT_VALUE_CHANGED, NULL);
  lv_obj_add_event_cb(s_ui.seek, on_seek_event, LV_EVENT_RELEASED, NULL);

  s_ui.time_pos = make_label(scr, &lv_font_montserrat_20, COLOR_TEXT_DIM);
  lv_label_set_text(s_ui.time_pos, "00:00");
  lv_obj_align_to(s_ui.time_pos, s_ui.seek, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 14);

  s_ui.time_total = make_label(scr, &lv_font_montserrat_20, COLOR_TEXT_DIM);
  lv_label_set_text(s_ui.time_total, "00:00");
  lv_obj_align_to(s_ui.time_total, s_ui.seek, LV_ALIGN_OUT_BOTTOM_RIGHT, 0, 14);
}

static void build_transport(lv_obj_t *scr)
{
  s_ui.btn_play = make_round_button(scr, 96, COLOR_ACCENT, COLOR_ON_ACCENT, LV_SYMBOL_PLAY, &lv_font_montserrat_40,
                                    &s_ui.btn_play_icon);
  lv_obj_align(s_ui.btn_play, LV_ALIGN_TOP_MID, 0, 300);
  lv_obj_add_event_cb(s_ui.btn_play, on_play_clicked, LV_EVENT_CLICKED, NULL);

  s_ui.btn_repeat = make_round_button(scr, 64, COLOR_SURFACE, COLOR_TEXT, LV_SYMBOL_LOOP, &lv_font_montserrat_28,
                                      &s_ui.btn_repeat_icon);
  lv_obj_align_to(s_ui.btn_repeat, s_ui.btn_play, LV_ALIGN_OUT_LEFT_MID, -48, 0);
  lv_obj_add_event_cb(s_ui.btn_repeat, on_repeat_clicked, LV_EVENT_CLICKED, NULL);

  s_ui.btn_next = make_round_button(scr, 64, COLOR_SURFACE, COLOR_TEXT, LV_SYMBOL_NEXT, &lv_font_montserrat_28, NULL);
  lv_obj_align_to(s_ui.btn_next, s_ui.btn_play, LV_ALIGN_OUT_RIGHT_MID, 48, 0);
  lv_obj_add_event_cb(s_ui.btn_next, on_next_clicked, LV_EVENT_CLICKED, NULL);
}

static void build_volume(lv_obj_t *scr, uint8_t volume)
{
  s_ui.volume = lv_slider_create(scr);
  lv_obj_set_width(s_ui.volume, 260);
  lv_slider_set_range(s_ui.volume, 0, 100);
  lv_slider_set_value(s_ui.volume, volume, LV_ANIM_OFF);
  style_slider(s_ui.volume, VOLUME_TOUCH_MARGIN);
  lv_obj_align(s_ui.volume, LV_ALIGN_BOTTOM_MID, 0, -42);
  lv_obj_add_event_cb(s_ui.volume, on_volume_event, LV_EVENT_VALUE_CHANGED, NULL);
  s_last_volume_sent = volume;

  s_ui.volume_icon = make_label(scr, &lv_font_montserrat_28, COLOR_TEXT_DIM);
  lv_label_set_text(s_ui.volume_icon, LV_SYMBOL_VOLUME_MAX);
  lv_obj_align_to(s_ui.volume_icon, s_ui.volume, LV_ALIGN_OUT_LEFT_MID, -24, 0);

  s_ui.volume_value = make_label(scr, &lv_font_montserrat_20, COLOR_TEXT_DIM);
  lv_label_set_text_fmt(s_ui.volume_value, "%u", (unsigned)volume);
  lv_obj_align_to(s_ui.volume_value, s_ui.volume, LV_ALIGN_OUT_RIGHT_MID, 24, 0);
}

void ui_init(void)
{
  lv_obj_t *scr = lv_screen_active();
  lv_obj_set_style_bg_color(scr, COLOR_BG, 0);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
  lv_obj_set_scrollable(scr, false);

  player_get_status(&s_status);

  build_header(scr);
  build_track_info(scr);
  build_seek_bar(scr);
  build_transport(scr);
  build_volume(scr, s_status.volume);

  refresh_cb(NULL);
  lv_timer_create(refresh_cb, REFRESH_PERIOD_MS, NULL);
}
