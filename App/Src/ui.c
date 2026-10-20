/**
 * @file  ui.c
 * @brief Player screen: song library on the left, "now playing" on the right.
 *
 *   +-----------------+------------------------------------------+
 *   | Library   60    | NOW PLAYING              12 / 60  44.1k  |
 *   |  > Song 1       |  +-----+  Artist                         |
 *   |    Song 2       |  |  ♪  |  Title                          |
 *   |    ...          |  +-----+                                 |
 *   |                 |  ●━━━━━━━━○─────────────────  seek       |
 *   |                 |     (↻)  (⏮)  ( ▶ )  (⏭)                 |
 *   |                 |  🔈 ━━━━━━○────────  20                  |
 *   +-----------------+------------------------------------------+
 *
 * The UI never touches the audio hardware: it sends commands through the
 * player API and refreshes itself from player_get_status() on a timer.
 * Song names are drawn with a TrueType font loaded from the SD card
 * (font.ttf / font.otf / font.ttc in the root) so that CJK names render
 * correctly; without it the built-in Latin font is used.
 * File names of the form "Artist - Title" are shown as artist and title.
 */
#include "ui.h"

#include <stdio.h>
#include <string.h>

#include "lvgl.h"
#include "log.h"
#include "player.h"

#define REFRESH_PERIOD_MS   100

/* Layout (800 x 480) */
#define LIBRARY_WIDTH       300
#define LIBRARY_HEADER_H    64
#define LIBRARY_ITEM_H      56
#define PANEL_X             LIBRARY_WIDTH
#define PANEL_WIDTH         (800 - LIBRARY_WIDTH)
#define PANEL_PAD           28
#define COVER_SIZE          104

/* Fonts loaded from the SD card */
#define TITLE_FONT_SIZE     32
#define TEXT_FONT_SIZE      22

/* Sliders: thick bar and a wide touch margin, fingers are not precise */
#define SLIDER_HEIGHT       14
#define SEEK_TOUCH_MARGIN   32
#define VOLUME_TOUCH_MARGIN 28

#define COLOR_BG            lv_color_hex(0xF4F6F9)
#define COLOR_PANEL         lv_color_hex(0xFFFFFF)
#define COLOR_DIVIDER       lv_color_hex(0xE2E6EC)
#define COLOR_SURFACE       lv_color_hex(0xE2E6EC)
#define COLOR_ACCENT        lv_color_hex(0x1A73E8)
#define COLOR_ACCENT_DARK   lv_color_hex(0x0B4FB3)
#define COLOR_ACCENT_SOFT   lv_color_hex(0xE8F0FE)
#define COLOR_TEXT          lv_color_hex(0x1F2328)
#define COLOR_TEXT_DIM      lv_color_hex(0x5F6875)
#define COLOR_WARNING       lv_color_hex(0xB45309)
#define COLOR_ON_ACCENT     lv_color_hex(0xFFFFFF)

static const char *const k_font_paths[] = { "S:/font.ttf", "S:/font.otf", "S:/font.ttc" };

typedef struct
{
  /* library */
  lv_obj_t *library_count;
  lv_obj_t *list;
  lv_obj_t *list_hint;
  /* now playing */
  lv_obj_t *track_info;     /* "12 / 60   44.1 kHz stereo" */
  lv_obj_t *cover;
  lv_obj_t *artist;
  lv_obj_t *title;
  lv_obj_t *message;
  lv_obj_t *seek;
  lv_obj_t *time_pos;
  lv_obj_t *time_total;
  lv_obj_t *btn_repeat;
  lv_obj_t *btn_repeat_icon;
  lv_obj_t *btn_prev;
  lv_obj_t *btn_play;
  lv_obj_t *btn_play_icon;
  lv_obj_t *btn_next;
  lv_obj_t *volume;
  lv_obj_t *volume_value;
} ui_widgets_t;

static ui_widgets_t s_ui;
static player_status_t s_status;              /* last status shown */
static uint32_t   s_shown_track_serial   = UINT32_MAX;
static uint32_t   s_shown_library_serial = UINT32_MAX;
static int32_t    s_highlighted = -1;          /* library row marked as playing */
static bool       s_first_refresh = true;
static int32_t    s_last_volume_sent = -1;
static bool       s_fonts_tried;
static lv_font_t *s_title_font;               /* NULL until loaded */
static lv_font_t *s_text_font;

static const lv_font_t *title_font(void) { return s_title_font ? s_title_font : &lv_font_montserrat_28; }
static const lv_font_t *text_font(void)  { return s_text_font ? s_text_font : &lv_font_montserrat_20; }

/* ---- helpers --------------------------------------------------------------- */

static void set_label_time(lv_obj_t *label, uint32_t seconds)
{
  lv_label_set_text_fmt(label, "%02lu:%02lu", (unsigned long)(seconds / 60U), (unsigned long)(seconds % 60U));
}

static lv_obj_t *make_label(lv_obj_t *parent, const lv_font_t *font, lv_color_t color)
{
  lv_obj_t *label = lv_label_create(parent);
  lv_obj_set_style_text_font(label, font, 0);
  lv_obj_set_style_text_color(label, color, 0);
  lv_label_set_text(label, "");
  return label;
}

/* A plain container without the theme's padding, border and scrolling */
static lv_obj_t *make_box(lv_obj_t *parent, lv_color_t bg)
{
  lv_obj_t *box = lv_obj_create(parent);
  lv_obj_set_style_bg_color(box, bg, 0);
  lv_obj_set_style_bg_opa(box, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(box, 0, 0);
  lv_obj_set_style_radius(box, 0, 0);
  lv_obj_set_style_pad_all(box, 0, 0);
  lv_obj_set_scrollable(box, false);
  return box;
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

/* "Artist - Title" -> artist / title. Without " - " the whole name is the title. */
static void split_name(const char *name, char *artist, size_t artist_len, const char **title)
{
  const char *sep = strstr(name, " - ");
  if (sep == NULL)
  {
    artist[0] = '\0';
    *title = name;
    return;
  }
  snprintf(artist, artist_len, "%.*s", (int)(sep - name), name);
  *title = sep + 3;
}

/* ---- event handlers -------------------------------------------------------- */

static void on_play_clicked(lv_event_t *e)   { (void)e; player_toggle_play(); }
static void on_next_clicked(lv_event_t *e)   { (void)e; player_next(); }
static void on_prev_clicked(lv_event_t *e)   { (void)e; player_prev(); }

static void on_repeat_clicked(lv_event_t *e)
{
  (void)e;
  player_set_repeat(s_status.repeat == PLAYER_REPEAT_ONE ? PLAYER_REPEAT_OFF : PLAYER_REPEAT_ONE);
}

static void on_track_clicked(lv_event_t *e)
{
  const uintptr_t index = (uintptr_t)lv_event_get_user_data(e);
  player_play_track((uint16_t)index);
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

/* ---- fonts ----------------------------------------------------------------- */

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
static void try_load_fonts(void)
{
  if (s_fonts_tried)
  {
    return;
  }
  s_fonts_tried = true;

  for (size_t i = 0; i < sizeof(k_font_paths) / sizeof(k_font_paths[0]); i++)
  {
    if (!file_exists(k_font_paths[i]))
    {
      continue;
    }
    s_title_font = lv_tiny_ttf_create_file(k_font_paths[i], TITLE_FONT_SIZE);
    s_text_font  = lv_tiny_ttf_create_file(k_font_paths[i], TEXT_FONT_SIZE);
    if (s_title_font == NULL || s_text_font == NULL)
    {
      LOG_WARN("ui: %s is not a usable TrueType/OpenType font", k_font_paths[i]);
      continue;
    }
    /* Glyphs missing from the font (e.g. LVGL symbols) come from the built-in one */
    s_title_font->fallback = &lv_font_montserrat_28;
    s_text_font->fallback  = &lv_font_montserrat_20;
    LOG_INFO("ui: font %s loaded", k_font_paths[i]);

    lv_obj_set_style_text_font(s_ui.title, s_title_font, 0);
    lv_obj_set_style_text_font(s_ui.artist, s_text_font, 0);
    /* Rebuild the library list with the new font if it already exists */
    s_shown_library_serial = UINT32_MAX;
    return;
  }
  LOG_WARN("ui: no font.ttf/font.otf/font.ttc on the SD card, non-Latin titles will not render");
}

/* ---- library list ---------------------------------------------------------- */

static void style_library_row(lv_obj_t *row, bool playing)
{
  lv_obj_t *label = lv_obj_get_child(row, 0);
  lv_obj_set_style_bg_color(row, playing ? COLOR_ACCENT_SOFT : COLOR_PANEL, 0);
  lv_obj_set_style_text_color(label, playing ? COLOR_ACCENT_DARK : COLOR_TEXT, 0);
  lv_label_set_text(lv_obj_get_child(row, 1), playing ? LV_SYMBOL_AUDIO : "");
}

static void build_library(uint16_t count)
{
  char name[PLAYER_TITLE_MAX];

  lv_obj_clean(s_ui.list);
  s_highlighted = -1;

  for (uint16_t i = 0; i < count; i++)
  {
    if (!player_get_track_name(i, name, sizeof(name)))
    {
      break;
    }
    lv_obj_t *row = make_box(s_ui.list, COLOR_PANEL);
    lv_obj_set_size(row, lv_pct(100), LIBRARY_ITEM_H);
    lv_obj_set_clickable(row, true);
    lv_obj_set_style_bg_color(row, COLOR_SURFACE, LV_STATE_PRESSED);
    lv_obj_set_style_border_side(row, LV_BORDER_SIDE_BOTTOM, 0);
    lv_obj_set_style_border_color(row, COLOR_DIVIDER, 0);
    lv_obj_set_style_border_width(row, 1, 0);
    lv_obj_add_event_cb(row, on_track_clicked, LV_EVENT_CLICKED, (void *)(uintptr_t)i);

    lv_obj_t *label = make_label(row, text_font(), COLOR_TEXT);
    lv_label_set_long_mode(label, LV_LABEL_LONG_MODE_DOTS);
    lv_obj_set_width(label, LIBRARY_WIDTH - 72);
    lv_label_set_text(label, name);
    lv_obj_align(label, LV_ALIGN_LEFT_MID, 20, 0);

    lv_obj_t *playing_icon = make_label(row, &lv_font_montserrat_20, COLOR_ACCENT);
    lv_obj_align(playing_icon, LV_ALIGN_RIGHT_MID, -18, 0);
  }

  lv_label_set_text_fmt(s_ui.library_count, "%u songs", (unsigned)count);
  lv_obj_set_hidden(s_ui.list_hint, count > 0U);
}

static void highlight_library_row(int32_t index)
{
  if (index == s_highlighted)
  {
    return;
  }
  const int32_t rows = (int32_t)lv_obj_get_child_count(s_ui.list);
  if (s_highlighted >= 0 && s_highlighted < rows)
  {
    style_library_row(lv_obj_get_child(s_ui.list, s_highlighted), false);
  }
  if (index >= 0 && index < rows)
  {
    lv_obj_t *row = lv_obj_get_child(s_ui.list, index);
    style_library_row(row, true);
    lv_obj_scroll_to_view(row, LV_ANIM_ON);
  }
  s_highlighted = index;
}

/* ---- refresh --------------------------------------------------------------- */

static void refresh_track(const player_status_t *st)
{
  char artist[PLAYER_TITLE_MAX];
  const char *title;

  split_name(st->title, artist, sizeof(artist), &title);
  lv_label_set_text(s_ui.artist, artist[0] != '\0' ? artist : "Unknown artist");
  lv_label_set_text(s_ui.title, title);

  lv_label_set_text_fmt(s_ui.track_info, "%u / %u   %lu.%lu kHz %s",
                        (unsigned)st->track_index + 1U, (unsigned)st->track_count,
                        (unsigned long)(st->sample_rate / 1000U),
                        (unsigned long)((st->sample_rate % 1000U) / 100U),
                        st->channels == 1U ? "mono" : "stereo");

  lv_slider_set_range(s_ui.seek, 0, (int32_t)(st->duration_s > 0U ? st->duration_s : 1U));
  set_label_time(s_ui.time_total, st->duration_s);

  highlight_library_row(st->track_index);
}

static void set_controls_enabled(bool enabled)
{
  lv_obj_t *const controls[] = { s_ui.btn_prev, s_ui.btn_play, s_ui.btn_next, s_ui.seek };
  for (size_t i = 0; i < sizeof(controls) / sizeof(controls[0]); i++)
  {
    if (enabled)
    {
      lv_obj_remove_state(controls[i], LV_STATE_DISABLED);
    }
    else
    {
      lv_obj_add_state(controls[i], LV_STATE_DISABLED);
    }
  }
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
    try_load_fonts();
  }

  if (st.library_serial != s_shown_library_serial)
  {
    s_shown_library_serial = st.library_serial;
    build_library(st.track_count);
    s_shown_track_serial = UINT32_MAX;   /* re-highlight the current track */
  }

  if (has_tracks && st.track_serial != s_shown_track_serial)
  {
    s_shown_track_serial = st.track_serial;
    refresh_track(&st);
  }

  /* User hint / error message replaces the song info */
  if (s_first_refresh || strcmp(st.message, s_status.message) != 0 || st.state != s_status.state)
  {
    s_first_refresh = false;
    const bool show_message = (st.message[0] != '\0');
    if (show_message)
    {
      lv_label_set_text_fmt(s_ui.message, LV_SYMBOL_WARNING "  %s", st.message);
    }
    lv_obj_set_hidden(s_ui.message, !show_message);
    lv_obj_set_hidden(s_ui.artist, show_message);
    lv_obj_set_hidden(s_ui.title, show_message);
    if (!has_tracks)
    {
      lv_label_set_text(s_ui.track_info, "");
      s_shown_track_serial = UINT32_MAX;
    }
  }

  lv_label_set_text(s_ui.btn_play_icon, st.state == PLAYER_STATE_PLAYING ? LV_SYMBOL_PAUSE : LV_SYMBOL_PLAY);
  set_controls_enabled(has_tracks);

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

static void build_library_panel(lv_obj_t *scr)
{
  lv_obj_t *panel = make_box(scr, COLOR_PANEL);
  lv_obj_set_size(panel, LIBRARY_WIDTH, 480);
  lv_obj_set_pos(panel, 0, 0);
  lv_obj_set_style_border_side(panel, LV_BORDER_SIDE_RIGHT, 0);
  lv_obj_set_style_border_color(panel, COLOR_DIVIDER, 0);
  lv_obj_set_style_border_width(panel, 1, 0);

  lv_obj_t *header = make_label(panel, &lv_font_montserrat_28, COLOR_TEXT);
  lv_label_set_text(header, "Library");
  lv_obj_set_pos(header, 20, 16);

  s_ui.library_count = make_label(panel, &lv_font_montserrat_20, COLOR_TEXT_DIM);
  lv_obj_align(s_ui.library_count, LV_ALIGN_TOP_RIGHT, -20, 22);

  s_ui.list = make_box(panel, COLOR_PANEL);
  lv_obj_set_size(s_ui.list, LIBRARY_WIDTH - 1, 480 - LIBRARY_HEADER_H);
  lv_obj_set_pos(s_ui.list, 0, LIBRARY_HEADER_H);
  lv_obj_set_scrollable(s_ui.list, true);
  lv_obj_set_scroll_dir(s_ui.list, LV_DIR_VER);
  lv_obj_set_flex_flow(s_ui.list, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_style_border_side(s_ui.list, LV_BORDER_SIDE_TOP, 0);
  lv_obj_set_style_border_color(s_ui.list, COLOR_DIVIDER, 0);
  lv_obj_set_style_border_width(s_ui.list, 1, 0);

  s_ui.list_hint = make_label(panel, &lv_font_montserrat_20, COLOR_TEXT_DIM);
  lv_label_set_text(s_ui.list_hint, "Put .wav files in /Music");
  lv_obj_align(s_ui.list_hint, LV_ALIGN_CENTER, 0, 0);
}

static void build_now_playing(lv_obj_t *scr)
{
  lv_obj_t *panel = make_box(scr, COLOR_BG);
  lv_obj_set_size(panel, PANEL_WIDTH, 480);
  lv_obj_set_pos(panel, PANEL_X, 0);

  /* Header */
  lv_obj_t *caption = make_label(panel, &lv_font_montserrat_20, COLOR_TEXT_DIM);
  lv_label_set_text(caption, "NOW PLAYING");
  lv_obj_set_style_text_letter_space(caption, 2, 0);
  lv_obj_set_pos(caption, PANEL_PAD, 22);

  /* One right-aligned label (LVGL re-aligns it whenever the text changes), so
   * the track position and format can never overlap each other or the caption */
  s_ui.track_info = make_label(panel, &lv_font_montserrat_20, COLOR_TEXT_DIM);
  lv_obj_align(s_ui.track_info, LV_ALIGN_TOP_RIGHT, -PANEL_PAD, 22);

  /* Cover tile */
  s_ui.cover = make_box(panel, COLOR_ACCENT);
  lv_obj_set_size(s_ui.cover, COVER_SIZE, COVER_SIZE);
  lv_obj_set_pos(s_ui.cover, PANEL_PAD, 70);
  lv_obj_set_style_radius(s_ui.cover, 16, 0);
  lv_obj_set_style_bg_grad_color(s_ui.cover, COLOR_ACCENT_DARK, 0);
  lv_obj_set_style_bg_grad_dir(s_ui.cover, LV_GRAD_DIR_VER, 0);
  lv_obj_t *note = make_label(s_ui.cover, &lv_font_montserrat_40, COLOR_ON_ACCENT);
  lv_label_set_text(note, LV_SYMBOL_AUDIO);
  lv_obj_center(note);

  /* Artist / title next to the cover */
  const int32_t text_x = PANEL_PAD + COVER_SIZE + 20;
  const int32_t text_w = PANEL_WIDTH - text_x - PANEL_PAD;

  s_ui.artist = make_label(panel, text_font(), COLOR_TEXT_DIM);
  lv_obj_set_width(s_ui.artist, text_w);
  lv_label_set_long_mode(s_ui.artist, LV_LABEL_LONG_MODE_DOTS);
  lv_obj_set_pos(s_ui.artist, text_x, 82);

  s_ui.title = make_label(panel, title_font(), COLOR_TEXT);
  lv_obj_set_width(s_ui.title, text_w);
  lv_label_set_long_mode(s_ui.title, LV_LABEL_LONG_MODE_SCROLL_CIRCULAR);
  lv_obj_set_pos(s_ui.title, text_x, 116);

  s_ui.message = make_label(panel, &lv_font_montserrat_20, COLOR_WARNING);
  lv_obj_set_width(s_ui.message, text_w);
  lv_label_set_long_mode(s_ui.message, LV_LABEL_LONG_MODE_WRAP);
  lv_obj_set_pos(s_ui.message, text_x, 100);
  lv_obj_set_hidden(s_ui.message, true);

  /* Seek bar */
  s_ui.seek = lv_slider_create(panel);
  lv_obj_set_width(s_ui.seek, PANEL_WIDTH - 2 * PANEL_PAD);
  lv_obj_set_pos(s_ui.seek, PANEL_PAD, 222);
  lv_slider_set_range(s_ui.seek, 0, 1);
  style_slider(s_ui.seek, SEEK_TOUCH_MARGIN);
  lv_obj_add_event_cb(s_ui.seek, on_seek_event, LV_EVENT_VALUE_CHANGED, NULL);
  lv_obj_add_event_cb(s_ui.seek, on_seek_event, LV_EVENT_RELEASED, NULL);

  s_ui.time_pos = make_label(panel, &lv_font_montserrat_20, COLOR_TEXT_DIM);
  lv_label_set_text(s_ui.time_pos, "00:00");
  lv_obj_align_to(s_ui.time_pos, s_ui.seek, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 12);

  s_ui.time_total = make_label(panel, &lv_font_montserrat_20, COLOR_TEXT_DIM);
  lv_label_set_text(s_ui.time_total, "00:00");
  lv_obj_align_to(s_ui.time_total, s_ui.seek, LV_ALIGN_OUT_BOTTOM_RIGHT, 0, 12);

  /* Transport: repeat, previous, play/pause, next */
  lv_obj_t *row = make_box(panel, COLOR_BG);
  lv_obj_set_size(row, PANEL_WIDTH, 96);
  lv_obj_set_pos(row, 0, 296);
  lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(row, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  lv_obj_set_style_pad_column(row, 28, 0);

  s_ui.btn_repeat = make_round_button(row, 56, COLOR_SURFACE, COLOR_TEXT, LV_SYMBOL_LOOP,
                                      &lv_font_montserrat_20, &s_ui.btn_repeat_icon);
  lv_obj_add_event_cb(s_ui.btn_repeat, on_repeat_clicked, LV_EVENT_CLICKED, NULL);

  s_ui.btn_prev = make_round_button(row, 64, COLOR_SURFACE, COLOR_TEXT, LV_SYMBOL_PREV,
                                    &lv_font_montserrat_28, NULL);
  lv_obj_add_event_cb(s_ui.btn_prev, on_prev_clicked, LV_EVENT_CLICKED, NULL);

  s_ui.btn_play = make_round_button(row, 88, COLOR_ACCENT, COLOR_ON_ACCENT, LV_SYMBOL_PLAY,
                                    &lv_font_montserrat_40, &s_ui.btn_play_icon);
  lv_obj_add_event_cb(s_ui.btn_play, on_play_clicked, LV_EVENT_CLICKED, NULL);

  s_ui.btn_next = make_round_button(row, 64, COLOR_SURFACE, COLOR_TEXT, LV_SYMBOL_NEXT,
                                    &lv_font_montserrat_28, NULL);
  lv_obj_add_event_cb(s_ui.btn_next, on_next_clicked, LV_EVENT_CLICKED, NULL);

  /* Keep the row symmetric around the play button: invisible spacer on the right */
  lv_obj_t *spacer = make_box(row, COLOR_BG);
  lv_obj_set_size(spacer, 56, 56);
  lv_obj_set_style_bg_opa(spacer, LV_OPA_TRANSP, 0);
}

static void build_volume(lv_obj_t *scr, uint8_t volume)
{
  lv_obj_t *icon = make_label(scr, &lv_font_montserrat_28, COLOR_TEXT_DIM);
  lv_label_set_text(icon, LV_SYMBOL_VOLUME_MAX);
  lv_obj_set_pos(icon, PANEL_X + PANEL_PAD, 420);

  s_ui.volume = lv_slider_create(scr);
  /* Leaves room for the icon on the left and "100" on the right inside the padding */
  lv_obj_set_width(s_ui.volume, PANEL_WIDTH - 2 * PANEL_PAD - 140);
  lv_slider_set_range(s_ui.volume, 0, 100);
  lv_slider_set_value(s_ui.volume, volume, LV_ANIM_OFF);
  style_slider(s_ui.volume, VOLUME_TOUCH_MARGIN);
  lv_obj_align_to(s_ui.volume, icon, LV_ALIGN_OUT_RIGHT_MID, 24, 0);
  lv_obj_add_event_cb(s_ui.volume, on_volume_event, LV_EVENT_VALUE_CHANGED, NULL);
  s_last_volume_sent = volume;

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

  build_library_panel(scr);
  build_now_playing(scr);
  build_volume(scr, s_status.volume);

  refresh_cb(NULL);
  lv_timer_create(refresh_cb, REFRESH_PERIOD_MS, NULL);
}

