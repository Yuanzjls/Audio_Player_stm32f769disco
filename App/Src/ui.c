/**
 * @file  ui.c
 * @brief Player screen: song library on the left, "now playing" on the right.
 *
 *   +-----------------+------------------------------------------+
 *   | Library   60    | NOW PLAYING              12 / 60  44.1k  |
 *   |  > Song 1       |  +---------+  Artist                     |
 *   |    Song 2       |  |         |  Title                      |
 *   |    ...          |  |  cover  |                             |
 *   |                 |  |         |  ▂▅▇▃▆▂▁▄▆▅▃▂▁  spectrum     |
 *   |                 |  +---------+                             |
 *   |                 |  ●━━━━━━━━○─────────────────  seek       |
 *   |                 |     (↻)  (⏮)  ( ▶ )  (⏭)  (⤮)            |
 *   |                 |  🔈 ━━━━━━○────────  20                  |
 *   +-----------------+------------------------------------------+
 *
 * The cover shows the picture embedded in the music file (decoded by the
 * hardware JPEG codec, see cover_art.c), or a note when there is none; a
 * blurred, washed-out copy of it fills the now-playing panel behind the rest.
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
#include "cover_art.h"
#include "log.h"
#include "player.h"
#include "spectrum.h"
#include "sysmon.h"

#define REFRESH_PERIOD_MS   100

/* Layout (800 x 480) */
#define LIBRARY_WIDTH       300
#define LIBRARY_HEADER_H    64
#define LIBRARY_ITEM_H      56
#define LIBRARY_LIST_H      (480 - LIBRARY_HEADER_H)
/* Row widgets that are recycled while scrolling: the visible rows plus margin */
#define LIBRARY_POOL        ((LIBRARY_LIST_H / LIBRARY_ITEM_H) + 3)
#define PANEL_X             LIBRARY_WIDTH
#define PANEL_WIDTH         (800 - LIBRARY_WIDTH)
#define PANEL_PAD           28
#define COVER_SIZE          COVER_ART_SIZE   /* 160 */
#define COVER_Y             64
#define ARTIST_Y            68
#define TITLE_Y             102
#define SPECTRUM_Y          (COVER_Y + COVER_SIZE - SPECTRUM_HEIGHT)   /* beside the cover, bottom-aligned */
#define SPECTRUM_HEIGHT     64
#define SEEK_Y              240
#define TRANSPORT_Y         302
#define SPECTRUM_PERIOD_MS  33      /* ~30 frames per second */
#define SPECTRUM_BAR_GAP    4
#define SPECTRUM_PEAK_H     3
#define SPECTRUM_MIN_BAR_H  2       /* a stub stays visible so the strip reads as an analyser at rest */
#define VOLUME_Y            420
#define FOLD_ANIM_MS        250
#define STATS_PERIOD_MS     1000

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
  /* panels */
  lv_obj_t *library;
  lv_obj_t *now_playing;
  lv_obj_t *btn_fold;
  lv_obj_t *caption;
  lv_obj_t *transport;
  lv_obj_t *volume_icon;
  lv_obj_t *stats;
  /* now playing */
  lv_obj_t *track_info;     /* "12 / 60   44.1 kHz stereo" */
  lv_obj_t *cover;
  lv_obj_t *cover_note;     /* shown when there is no picture */
  lv_obj_t *cover_image;
  lv_obj_t *backdrop;       /* blurred cover behind the now-playing panel */
  lv_obj_t *artist;
  lv_obj_t *title;
  lv_obj_t *message;
  lv_obj_t *spectrum;
  lv_obj_t *seek;
  lv_obj_t *time_pos;
  lv_obj_t *time_total;
  lv_obj_t *btn_repeat;
  lv_obj_t *btn_repeat_icon;
  lv_obj_t *btn_shuffle;
  lv_obj_t *btn_shuffle_icon;
  lv_obj_t *btn_prev;
  lv_obj_t *btn_play;
  lv_obj_t *btn_play_icon;
  lv_obj_t *btn_next;
  lv_obj_t *volume;
  lv_obj_t *volume_value;
} ui_widgets_t;

static ui_widgets_t s_ui;
static bool         s_library_folded;
static spectrum_t   s_spectrum;

/* Bar and peak heights in pixels as last drawn (a peak of 0 is hidden) */
static int16_t s_bar_px[SPECTRUM_BANDS];
static int16_t s_peak_px[SPECTRUM_BANDS];
static player_status_t s_status;              /* last status shown */
static uint32_t   s_shown_track_serial   = UINT32_MAX;
static uint32_t   s_shown_library_serial = UINT32_MAX;
static int32_t    s_highlighted = -1;          /* library index marked as playing */

/* Virtualized library: s_rows[k] shows the track index i with i % LIBRARY_POOL == k,
 * so scrolling by one row re-labels just one widget, and only the names that
 * become visible are measured and drawn (glyph lookups are slow on the SD card). */
static lv_obj_t  *s_rows[LIBRARY_POOL];
static int32_t    s_row_index[LIBRARY_POOL];     /* track shown by each row, -1 = none */
static uint16_t   s_library_count;
static lv_obj_t  *s_list_spacer;
static bool       s_first_refresh = true;
static uint32_t   s_shown_position = UINT32_MAX;   /* seconds shown by the position label */
static int32_t    s_last_volume_sent = -1;
static uint32_t   s_cover_id = UINT32_MAX;     /* track serial whose cover is wanted */
static bool       s_cover_done = true;
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

static void on_shuffle_clicked(lv_event_t *e)
{
  (void)e;
  player_set_shuffle(!s_status.shuffle);
}

static void on_track_clicked(lv_event_t *e)
{
  const uintptr_t slot = (uintptr_t)lv_event_get_user_data(e);
  if (s_row_index[slot] >= 0)
  {
    player_play_track((uint16_t)s_row_index[slot]);
  }
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
    /* Re-label the library rows with the new font */
    for (uint32_t k = 0; k < LIBRARY_POOL; k++)
    {
      lv_obj_set_style_text_font(lv_obj_get_child(s_rows[k], 0), s_text_font, 0);
      s_row_index[k] = -1;
    }
    s_shown_library_serial = UINT32_MAX;
    return;
  }
  LOG_WARN("ui: no font.ttf/font.otf/font.ttc on the SD card, non-Latin titles will not render");
}

/* ---- library list (virtualized) ------------------------------------------ */

static void style_library_row(lv_obj_t *row, bool playing)
{
  lv_obj_t *label = lv_obj_get_child(row, 0);
  lv_obj_set_style_bg_color(row, playing ? COLOR_ACCENT_SOFT : COLOR_PANEL, 0);
  lv_obj_set_style_text_color(label, playing ? COLOR_ACCENT_DARK : COLOR_TEXT, 0);
  lv_label_set_text(lv_obj_get_child(row, 1), playing ? LV_SYMBOL_AUDIO : "");
}

/* Show track @p index in row slot @p slot (or hide the row) */
static void bind_row(uint32_t slot, int32_t index)
{
  lv_obj_t *row = s_rows[slot];

  if (index < 0 || index >= (int32_t)s_library_count)
  {
    lv_obj_set_hidden(row, true);
    s_row_index[slot] = -1;
    return;
  }
  if (s_row_index[slot] != index)
  {
    char name[PLAYER_TITLE_MAX];
    if (!player_get_track_name((uint16_t)index, name, sizeof(name)))
    {
      name[0] = '\0';
    }
    lv_label_set_text(lv_obj_get_child(row, 0), name);
    lv_obj_set_y(row, index * LIBRARY_ITEM_H);
    s_row_index[slot] = index;
  }
  style_library_row(row, index == s_highlighted);
  lv_obj_set_hidden(row, false);
}

/* Bind the rows to the tracks around the current scroll position */
static void layout_rows(void)
{
  int32_t first = lv_obj_get_scroll_y(s_ui.list) / LIBRARY_ITEM_H;
  if (first < 0)
  {
    first = 0;
  }
  for (int32_t i = first; i < first + (int32_t)LIBRARY_POOL; i++)
  {
    bind_row((uint32_t)i % LIBRARY_POOL, i);
  }
}

static void on_list_scroll(lv_event_t *e)
{
  (void)e;
  layout_rows();
}

static void build_library(uint16_t count)
{
  s_library_count = count;
  s_highlighted   = -1;
  lv_obj_set_height(s_list_spacer, (int32_t)count * LIBRARY_ITEM_H);
  lv_obj_scroll_to_y(s_ui.list, 0, LV_ANIM_OFF);
  for (uint32_t k = 0; k < LIBRARY_POOL; k++)
  {
    s_row_index[k] = -1;
  }
  layout_rows();

  lv_label_set_text_fmt(s_ui.library_count, "%u songs", (unsigned)count);
  lv_obj_set_hidden(s_ui.list_hint, count > 0U);
}

static void highlight_library_row(int32_t index)
{
  if (index == s_highlighted)
  {
    return;
  }
  s_highlighted = index;

  /* Bring the playing track into view if it is not (fully) visible */
  const int32_t top    = index * LIBRARY_ITEM_H;
  const int32_t scroll = lv_obj_get_scroll_y(s_ui.list);
  if (top < scroll || top + LIBRARY_ITEM_H > scroll + LIBRARY_LIST_H)
  {
    lv_obj_scroll_to_y(s_ui.list, (top > LIBRARY_ITEM_H) ? top - LIBRARY_ITEM_H : 0, LV_ANIM_ON);
  }
  /* Restyle the visible rows (the scroll event re-binds them as it animates) */
  for (uint32_t k = 0; k < LIBRARY_POOL; k++)
  {
    if (s_row_index[k] >= 0)
    {
      style_library_row(s_rows[k], s_row_index[k] == s_highlighted);
    }
  }
}

/* ---- cover ----------------------------------------------------------------- */

/* The picture (rounded like the tile) in place of the note and its blurred copy
 * behind the panel, or (NULL) the note and the plain panel back */
static void show_cover(const lv_image_dsc_t *image, const lv_image_dsc_t *backdrop)
{
  if (image != NULL)
  {
    lv_image_set_src(s_ui.cover_image, image);
    lv_image_set_src(s_ui.backdrop, backdrop);
  }
  lv_obj_set_hidden(s_ui.cover_image, image == NULL);
  lv_obj_set_hidden(s_ui.backdrop, image == NULL);
  lv_obj_set_hidden(s_ui.cover_note, image != NULL);
  lv_obj_set_style_bg_opa(s_ui.cover, image != NULL ? LV_OPA_TRANSP : LV_OPA_COVER, 0);
}

/* Ask the cover task for the picture of the new track. The previous cover
 * stays until the result is known (no flash of the plain panel in between). */
static void request_cover(const player_status_t *st)
{
  char path[sizeof(PLAYER_MUSIC_DIR) + PLAYER_TITLE_MAX + 1];

  s_cover_id   = st->track_serial;
  s_cover_done = false;
  if (!player_get_track_path(st->track_index, path, sizeof(path)))
  {
    s_cover_done = true;
    show_cover(NULL, NULL);
    return;
  }
  cover_art_request(st->track_serial, path, st->art_format, st->art_offset, st->art_size);
}

static void poll_cover(void)
{
  if (s_cover_done)
  {
    return;
  }
  const lv_image_dsc_t *image    = NULL;
  const lv_image_dsc_t *backdrop = NULL;
  const cover_art_state_t state = cover_art_poll(s_cover_id, &image, &backdrop);
  if (state != COVER_ART_PENDING)
  {
    s_cover_done = true;
    show_cover(state == COVER_ART_READY ? image : NULL, backdrop);
  }
}

/* ---- refresh --------------------------------------------------------------- */

static void set_duration(uint32_t duration_s)
{
  lv_slider_set_range(s_ui.seek, 0, (int32_t)(duration_s > 0U ? duration_s : 1U));
  set_label_time(s_ui.time_total, duration_s);
}

static void refresh_track(const player_status_t *st)
{
  char artist[PLAYER_TITLE_MAX];
  const char *title;

  if (st->artist[0] != '\0')
  {
    /* Tags present */
    snprintf(artist, sizeof(artist), "%s", st->artist);
    title = st->title;
  }
  else
  {
    split_name(st->title, artist, sizeof(artist), &title);
  }
  lv_label_set_text(s_ui.artist, artist[0] != '\0' ? artist : "Unknown artist");
  lv_label_set_text(s_ui.title, title);

  /* Kept short: it shares the header line with the "NOW PLAYING" caption */
  lv_label_set_text_fmt(s_ui.track_info, "%u / %u   %s  %lu.%lu kHz",
                        (unsigned)st->track_index + 1U, (unsigned)st->track_count, st->format,
                        (unsigned long)(st->sample_rate / 1000U),
                        (unsigned long)((st->sample_rate % 1000U) / 100U));

  set_duration(st->duration_s);
  highlight_library_row(st->track_index);
  request_cover(st);
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
  const bool first      = s_first_refresh;

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
  if (first || strcmp(st.message, s_status.message) != 0 || st.state != s_status.state)
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

  /* Setting a text or style redraws the widget even if nothing changed, so only
   * touch widgets whose value did change */
  if (first || st.state != s_status.state)
  {
    lv_label_set_text(s_ui.btn_play_icon, st.state == PLAYER_STATE_PLAYING ? LV_SYMBOL_PAUSE : LV_SYMBOL_PLAY);
  }
  set_controls_enabled(has_tracks);   /* no-op if the state is unchanged */

  if (first || st.repeat != s_status.repeat)
  {
    const bool repeat_one = (st.repeat == PLAYER_REPEAT_ONE);
    lv_obj_set_style_bg_color(s_ui.btn_repeat, repeat_one ? COLOR_ACCENT : COLOR_SURFACE, 0);
    lv_obj_set_style_text_color(s_ui.btn_repeat_icon, repeat_one ? COLOR_ON_ACCENT : COLOR_TEXT, 0);
  }
  if (first || st.shuffle != s_status.shuffle)
  {
    lv_obj_set_style_bg_color(s_ui.btn_shuffle, st.shuffle ? COLOR_ACCENT : COLOR_SURFACE, 0);
    lv_obj_set_style_text_color(s_ui.btn_shuffle_icon, st.shuffle ? COLOR_ON_ACCENT : COLOR_TEXT, 0);
  }

  /* The exact length of a VBR MP3 may be known only after a while */
  if (has_tracks && st.track_serial == s_status.track_serial && st.duration_s != s_status.duration_s)
  {
    set_duration(st.duration_s);
  }

  /* Progress, unless the user is dragging the seek bar */
  if (!lv_slider_is_dragged(s_ui.seek))
  {
    lv_slider_set_value(s_ui.seek, (int32_t)st.position_s, LV_ANIM_OFF);   /* no-op if unchanged */
    if (st.position_s != s_shown_position)
    {
      s_shown_position = st.position_s;
      set_label_time(s_ui.time_pos, st.position_s);
    }
  }
  else
  {
    s_shown_position = UINT32_MAX;   /* the label follows the drag meanwhile */
  }

  s_status = st;
  poll_cover();
}

/* ---- spectrum ---------------------------------------------------------------- */

typedef struct
{
  int32_t x0;       /* left edge of bar 0 */
  int32_t slot;     /* bar pitch */
  int32_t bar_w;
  int32_t bottom;   /* y of the bottom row */
  int32_t usable;   /* tallest bar, leaving room for a peak marker */
} spectrum_geom_t;

static spectrum_geom_t spectrum_geometry(void)
{
  lv_area_t box;
  lv_obj_get_coords(s_ui.spectrum, &box);

  const int32_t width = lv_area_get_width(&box);
  spectrum_geom_t g;
  g.slot   = width / SPECTRUM_BANDS;
  g.bar_w  = g.slot - SPECTRUM_BAR_GAP;
  g.x0     = box.x1 + (width - g.slot * SPECTRUM_BANDS + SPECTRUM_BAR_GAP) / 2;
  g.bottom = box.y2;
  g.usable = lv_area_get_height(&box) - SPECTRUM_PEAK_H - 2;
  return g;
}

static void on_spectrum_draw(lv_event_t *e)
{
  lv_layer_t *layer = lv_event_get_layer(e);
  const spectrum_geom_t g = spectrum_geometry();

  lv_draw_rect_dsc_t bar;
  lv_draw_rect_dsc_init(&bar);
  /* Solid, square bars: gradients and rounded corners cost several ms per frame */
  bar.bg_color = COLOR_ACCENT;
  bar.radius   = 0;

  lv_draw_rect_dsc_t peak;
  lv_draw_rect_dsc_init(&peak);
  peak.bg_color = COLOR_ACCENT_DARK;
  peak.radius   = 0;

  /* Skip bars outside the area being redrawn */
  const lv_area_t *clip = &layer->_clip_area;
  int32_t first = (clip->x1 - g.x0) / g.slot;
  int32_t last  = (clip->x2 - g.x0) / g.slot;
  first = LV_CLAMP(0, first, SPECTRUM_BANDS - 1);
  last  = LV_CLAMP(0, last, SPECTRUM_BANDS - 1);

  for (int32_t b = first; b <= last; b++)
  {
    const int32_t x = g.x0 + b * g.slot;
    const int32_t h = s_bar_px[b];
    const lv_area_t a = { x, g.bottom - h + 1, x + g.bar_w - 1, g.bottom };
    lv_draw_rect(layer, &bar, &a);

    const int32_t p = s_peak_px[b];
    if (p > 0)
    {
      const lv_area_t c = { x, g.bottom - p - SPECTRUM_PEAK_H + 1, x + g.bar_w - 1, g.bottom - p };
      lv_draw_rect(layer, &peak, &c);
    }
  }
}

/* Rows (as heights above the bottom, inclusive) that differ between two states of a bar */
static void spectrum_dirty_rows(int32_t old_bar, int32_t old_peak, int32_t new_bar, int32_t new_peak,
                                int32_t *lo, int32_t *hi)
{
  *lo = INT32_MAX;
  *hi = INT32_MIN;
  if (old_bar != new_bar)
  {
    *lo = LV_MIN(old_bar, new_bar);
    *hi = LV_MAX(old_bar, new_bar) - 1;
  }
  if (old_peak != new_peak)
  {
    const int32_t peaks[2] = { old_peak, new_peak };
    for (uint32_t i = 0; i < 2U; i++)
    {
      if (peaks[i] > 0)
      {
        *lo = LV_MIN(*lo, peaks[i]);
        *hi = LV_MAX(*hi, peaks[i] + SPECTRUM_PEAK_H - 1);
      }
    }
  }
}

static void spectrum_cb(lv_timer_t *timer)
{
  (void)timer;
  if (!spectrum_update(&s_spectrum, s_status.sample_rate, lv_tick_get()))
  {
    return;
  }

  /* Redraw only the rows that changed: one area across all bars. (One area per
   * bar would redraw fewer pixels but costs more: every area has a fixed cost.) */
  const spectrum_geom_t g = spectrum_geometry();
  int32_t dirty_lo = INT32_MAX;
  int32_t dirty_hi = INT32_MIN;

  for (int32_t b = 0; b < SPECTRUM_BANDS; b++)
  {
    int32_t bar = (int32_t)(s_spectrum.level[b] * (float)g.usable);
    bar = LV_CLAMP(SPECTRUM_MIN_BAR_H, bar, g.usable);
    int32_t peak = (int32_t)(s_spectrum.peak[b] * (float)g.usable);
    peak = (peak > bar + 1) ? LV_MIN(peak, g.usable) : 0;

    int32_t lo;
    int32_t hi;
    spectrum_dirty_rows(s_bar_px[b], s_peak_px[b], bar, peak, &lo, &hi);
    dirty_lo = LV_MIN(dirty_lo, lo);
    dirty_hi = LV_MAX(dirty_hi, hi);
    s_bar_px[b]  = (int16_t)bar;
    s_peak_px[b] = (int16_t)peak;
  }

  if (dirty_lo <= dirty_hi)
  {
    const lv_area_t area = { g.x0, g.bottom - dirty_hi, g.x0 + (SPECTRUM_BANDS - 1) * g.slot + g.bar_w - 1,
                             g.bottom - dirty_lo };
    lv_obj_invalidate_area(s_ui.spectrum, &area);
  }
}

/* ---- construction ---------------------------------------------------------- */

static void on_fold_clicked(lv_event_t *e);
static void layout_now_playing(int32_t w);

static void build_library_panel(lv_obj_t *scr)
{
  lv_obj_t *panel = make_box(scr, COLOR_PANEL);
  s_ui.library = panel;
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
  lv_obj_set_size(s_ui.list, LIBRARY_WIDTH - 1, LIBRARY_LIST_H);
  lv_obj_set_pos(s_ui.list, 0, LIBRARY_HEADER_H);
  lv_obj_set_scrollable(s_ui.list, true);
  lv_obj_set_scroll_dir(s_ui.list, LV_DIR_VER);
  lv_obj_set_style_border_side(s_ui.list, LV_BORDER_SIDE_TOP, 0);
  lv_obj_set_style_border_color(s_ui.list, COLOR_DIVIDER, 0);
  lv_obj_set_style_border_width(s_ui.list, 1, 0);
  lv_obj_add_event_cb(s_ui.list, on_list_scroll, LV_EVENT_SCROLL, NULL);

  /* Invisible, as tall as the whole library: gives the list its scroll range */
  s_list_spacer = make_box(s_ui.list, COLOR_PANEL);
  lv_obj_set_size(s_list_spacer, 1, 0);
  lv_obj_set_style_bg_opa(s_list_spacer, LV_OPA_TRANSP, 0);

  for (uint32_t k = 0; k < LIBRARY_POOL; k++)
  {
    lv_obj_t *row = make_box(s_ui.list, COLOR_PANEL);
    lv_obj_set_size(row, LIBRARY_WIDTH - 1, LIBRARY_ITEM_H);
    lv_obj_set_clickable(row, true);
    lv_obj_set_style_bg_color(row, COLOR_SURFACE, LV_STATE_PRESSED);
    lv_obj_set_style_border_side(row, LV_BORDER_SIDE_BOTTOM, 0);
    lv_obj_set_style_border_color(row, COLOR_DIVIDER, 0);
    lv_obj_set_style_border_width(row, 1, 0);
    lv_obj_add_event_cb(row, on_track_clicked, LV_EVENT_CLICKED, (void *)(uintptr_t)k);

    lv_obj_t *label = make_label(row, text_font(), COLOR_TEXT);
    lv_label_set_long_mode(label, LV_LABEL_LONG_MODE_DOTS);
    lv_obj_set_width(label, LIBRARY_WIDTH - 72);
    lv_obj_align(label, LV_ALIGN_LEFT_MID, 20, 0);

    lv_obj_t *playing_icon = make_label(row, &lv_font_montserrat_20, COLOR_ACCENT);
    lv_obj_align(playing_icon, LV_ALIGN_RIGHT_MID, -18, 0);

    lv_obj_set_hidden(row, true);
    s_rows[k]      = row;
    s_row_index[k] = -1;
  }

  s_ui.list_hint = make_label(panel, &lv_font_montserrat_20, COLOR_TEXT_DIM);
  lv_label_set_text(s_ui.list_hint, "Put .wav files in /Music");
  lv_obj_align(s_ui.list_hint, LV_ALIGN_CENTER, 0, 0);
}

static void build_now_playing(lv_obj_t *scr)
{
  lv_obj_t *panel = make_box(scr, COLOR_BG);
  lv_obj_set_size(panel, PANEL_WIDTH, 480);
  lv_obj_set_pos(panel, PANEL_X, 0);
  s_ui.now_playing = panel;

  /* Backdrop: the blurred cover behind everything else (first child), centred so
   * that it fills the panel at either width (library shown or folded away) */
  s_ui.backdrop = lv_image_create(panel);
  lv_obj_align(s_ui.backdrop, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_hidden(s_ui.backdrop, true);

  /* Header: library fold button, caption */
  s_ui.btn_fold = make_box(panel, COLOR_BG);
  lv_obj_set_style_bg_opa(s_ui.btn_fold, LV_OPA_TRANSP, 0);
  lv_obj_set_style_bg_opa(s_ui.btn_fold, LV_OPA_COVER, LV_STATE_PRESSED);
  lv_obj_set_size(s_ui.btn_fold, 44, 40);
  lv_obj_set_pos(s_ui.btn_fold, PANEL_PAD - 10, 12);
  lv_obj_set_clickable(s_ui.btn_fold, true);
  lv_obj_set_ext_click_area(s_ui.btn_fold, 10);
  lv_obj_set_style_radius(s_ui.btn_fold, 8, 0);
  lv_obj_set_style_bg_color(s_ui.btn_fold, COLOR_SURFACE, LV_STATE_PRESSED);
  lv_obj_add_event_cb(s_ui.btn_fold, on_fold_clicked, LV_EVENT_CLICKED, NULL);
  lv_obj_t *fold_icon = make_label(s_ui.btn_fold, &lv_font_montserrat_28, COLOR_TEXT_DIM);
  lv_label_set_text(fold_icon, LV_SYMBOL_LIST);
  lv_obj_center(fold_icon);

  s_ui.caption = make_label(panel, &lv_font_montserrat_20, COLOR_TEXT_DIM);
  lv_label_set_text(s_ui.caption, "NOW PLAYING");
  lv_obj_set_style_text_letter_space(s_ui.caption, 2, 0);
  lv_obj_set_pos(s_ui.caption, PANEL_PAD + 44, 22);

  /* One right-aligned label (LVGL re-aligns it whenever the text changes), so
   * the track position and format can never overlap each other or the caption */
  s_ui.track_info = make_label(panel, &lv_font_montserrat_20, COLOR_TEXT_DIM);
  lv_obj_align(s_ui.track_info, LV_ALIGN_TOP_RIGHT, -PANEL_PAD, 22);

  /* Cover tile */
  s_ui.cover = make_box(panel, COLOR_ACCENT);
  lv_obj_set_size(s_ui.cover, COVER_SIZE, COVER_SIZE);
  lv_obj_set_pos(s_ui.cover, PANEL_PAD, COVER_Y);
  lv_obj_set_style_radius(s_ui.cover, 16, 0);
  lv_obj_set_style_bg_grad_color(s_ui.cover, COLOR_ACCENT_DARK, 0);
  lv_obj_set_style_bg_grad_dir(s_ui.cover, LV_GRAD_DIR_VER, 0);
  s_ui.cover_note = make_label(s_ui.cover, &lv_font_montserrat_40, COLOR_ON_ACCENT);
  lv_label_set_text(s_ui.cover_note, LV_SYMBOL_AUDIO);
  lv_obj_center(s_ui.cover_note);
  s_ui.cover_image = lv_image_create(s_ui.cover);
  lv_obj_center(s_ui.cover_image);
  lv_obj_set_hidden(s_ui.cover_image, true);

  /* Artist / title next to the cover (widths set in layout_now_playing) */
  const int32_t text_x = PANEL_PAD + COVER_SIZE + 20;

  s_ui.artist = make_label(panel, text_font(), COLOR_TEXT_DIM);
  lv_label_set_long_mode(s_ui.artist, LV_LABEL_LONG_MODE_DOTS);
  lv_obj_set_pos(s_ui.artist, text_x, ARTIST_Y);

  s_ui.title = make_label(panel, title_font(), COLOR_TEXT);
  lv_label_set_long_mode(s_ui.title, LV_LABEL_LONG_MODE_SCROLL_CIRCULAR);
  lv_obj_set_pos(s_ui.title, text_x, TITLE_Y);

  s_ui.message = make_label(panel, &lv_font_montserrat_20, COLOR_WARNING);
  lv_label_set_long_mode(s_ui.message, LV_LABEL_LONG_MODE_WRAP);
  lv_obj_set_pos(s_ui.message, text_x, ARTIST_Y + 14);
  lv_obj_set_hidden(s_ui.message, true);

  /* Spectrum strip beside the cover (drawn in on_spectrum_draw). Transparent: the
   * backdrop shows through; redrawing it under the bars is a plain copy. */
  s_ui.spectrum = make_box(panel, COLOR_BG);
  lv_obj_set_style_bg_opa(s_ui.spectrum, LV_OPA_TRANSP, 0);
  lv_obj_set_height(s_ui.spectrum, SPECTRUM_HEIGHT);
  lv_obj_set_pos(s_ui.spectrum, text_x, SPECTRUM_Y);
  lv_obj_add_event_cb(s_ui.spectrum, on_spectrum_draw, LV_EVENT_DRAW_MAIN, NULL);
  for (uint32_t b = 0; b < SPECTRUM_BANDS; b++)
  {
    s_bar_px[b] = SPECTRUM_MIN_BAR_H;
  }

  /* Seek bar */
  s_ui.seek = lv_slider_create(panel);
  lv_obj_set_pos(s_ui.seek, PANEL_PAD, SEEK_Y);
  lv_slider_set_range(s_ui.seek, 0, 1);
  style_slider(s_ui.seek, SEEK_TOUCH_MARGIN);
  lv_obj_add_event_cb(s_ui.seek, on_seek_event, LV_EVENT_VALUE_CHANGED, NULL);
  lv_obj_add_event_cb(s_ui.seek, on_seek_event, LV_EVENT_RELEASED, NULL);

  s_ui.time_pos = make_label(panel, &lv_font_montserrat_20, COLOR_TEXT_DIM);
  lv_label_set_text(s_ui.time_pos, "00:00");


  s_ui.time_total = make_label(panel, &lv_font_montserrat_20, COLOR_TEXT_DIM);
  lv_label_set_text(s_ui.time_total, "00:00");


  /* Transport: repeat, previous, play/pause, next, shuffle */
  lv_obj_t *row = make_box(panel, COLOR_BG);
  lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
  s_ui.transport = row;
  lv_obj_set_height(row, 96);
  lv_obj_set_pos(row, 0, TRANSPORT_Y);
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

  s_ui.btn_shuffle = make_round_button(row, 56, COLOR_SURFACE, COLOR_TEXT, LV_SYMBOL_SHUFFLE,
                                       &lv_font_montserrat_20, &s_ui.btn_shuffle_icon);
  lv_obj_add_event_cb(s_ui.btn_shuffle, on_shuffle_clicked, LV_EVENT_CLICKED, NULL);
}

static void build_volume(lv_obj_t *panel, uint8_t volume)
{
  s_ui.volume_icon = make_label(panel, &lv_font_montserrat_28, COLOR_TEXT_DIM);
  lv_label_set_text(s_ui.volume_icon, LV_SYMBOL_VOLUME_MAX);
  lv_obj_set_pos(s_ui.volume_icon, PANEL_PAD, VOLUME_Y);

  s_ui.volume = lv_slider_create(panel);
  lv_slider_set_range(s_ui.volume, 0, 100);
  lv_slider_set_value(s_ui.volume, volume, LV_ANIM_OFF);
  style_slider(s_ui.volume, VOLUME_TOUCH_MARGIN);
  lv_obj_add_event_cb(s_ui.volume, on_volume_event, LV_EVENT_VALUE_CHANGED, NULL);
  s_last_volume_sent = volume;

  s_ui.volume_value = make_label(panel, &lv_font_montserrat_20, COLOR_TEXT_DIM);
  lv_label_set_text_fmt(s_ui.volume_value, "%u", (unsigned)volume);

  /* System status line (CPU load, memory) at the bottom right */
  s_ui.stats = make_label(panel, &lv_font_montserrat_14, COLOR_TEXT_DIM);
  lv_obj_align(s_ui.stats, LV_ALIGN_BOTTOM_RIGHT, -PANEL_PAD, -6);
}

/* Size / place everything in the now-playing panel for a panel width of @p w */
static void layout_now_playing(int32_t w)
{
  const int32_t text_x = PANEL_PAD + COVER_SIZE + 20;
  const int32_t text_w = w - text_x - PANEL_PAD;
  const int32_t full_w = w - 2 * PANEL_PAD;

  lv_obj_set_width(s_ui.artist, text_w);
  lv_obj_set_width(s_ui.title, text_w);
  lv_obj_set_width(s_ui.message, text_w);
  lv_obj_set_width(s_ui.spectrum, text_w);
  lv_obj_set_width(s_ui.seek, full_w);
  lv_obj_set_width(s_ui.transport, w);
  /* Volume: room for the icon on the left and "100" on the right inside the padding */
  lv_obj_set_width(s_ui.volume, full_w - 140);

  lv_obj_update_layout(s_ui.now_playing);
  lv_obj_align_to(s_ui.time_pos, s_ui.seek, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 12);
  lv_obj_align_to(s_ui.time_total, s_ui.seek, LV_ALIGN_OUT_BOTTOM_RIGHT, 0, 12);
  lv_obj_align_to(s_ui.volume, s_ui.volume_icon, LV_ALIGN_OUT_RIGHT_MID, 24, 0);
  lv_obj_align_to(s_ui.volume_value, s_ui.volume, LV_ALIGN_OUT_RIGHT_MID, 24, 0);
}

/* @p offset: 0 = library shown, -LIBRARY_WIDTH = folded away */
static void set_library_offset(void *var, int32_t offset)
{
  (void)var;
  const int32_t w = PANEL_WIDTH - offset;
  lv_obj_set_x(s_ui.library, offset);
  lv_obj_set_x(s_ui.now_playing, LIBRARY_WIDTH + offset);
  lv_obj_set_width(s_ui.now_playing, w);
  layout_now_playing(w);
}

static void on_fold_clicked(lv_event_t *e)
{
  (void)e;
  s_library_folded = !s_library_folded;

  lv_anim_t a;
  lv_anim_init(&a);
  lv_anim_set_var(&a, s_ui.library);
  lv_anim_set_exec_cb(&a, set_library_offset);
  lv_anim_set_values(&a, lv_obj_get_x(s_ui.library), s_library_folded ? -LIBRARY_WIDTH : 0);
  lv_anim_set_duration(&a, FOLD_ANIM_MS);
  lv_anim_set_path_cb(&a, lv_anim_path_ease_in_out);
  lv_anim_start(&a);
}

static void stats_cb(lv_timer_t *timer)
{
  (void)timer;
  uint32_t heap_used, heap_total;
  lv_mem_monitor_t gui;

  sysmon_rtos_heap(&heap_used, &heap_total);
  lv_mem_monitor(&gui);
  const uint32_t gui_used_kb  = (uint32_t)((gui.total_size - gui.free_size) / 1024U);
  const uint32_t gui_total_kb = (uint32_t)(gui.total_size / 1024U);

  lv_label_set_text_fmt(s_ui.stats, "CPU %" LV_PRIu32 "%%    Heap %" LV_PRIu32 "/%" LV_PRIu32 " KB    GUI %" LV_PRIu32 "/%" LV_PRIu32 " KB",
                        sysmon_cpu_load(), heap_used / 1024U, heap_total / 1024U, gui_used_kb, gui_total_kb);
}

void ui_init(void)
{
  lv_obj_t *scr = lv_screen_active();
  lv_obj_set_style_bg_color(scr, COLOR_BG, 0);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
  lv_obj_set_scrollable(scr, false);

  player_get_status(&s_status);
  cover_art_init(0xF4F6F9);   /* COLOR_BG: backdrops are washed towards the panel colour */

  build_library_panel(scr);
  build_now_playing(scr);
  build_volume(s_ui.now_playing, s_status.volume);
  layout_now_playing(PANEL_WIDTH);

  /* Last child of the panel: nothing above it needs redrawing with its bars */
  lv_obj_move_foreground(s_ui.spectrum);

  refresh_cb(NULL);
  lv_timer_create(refresh_cb, REFRESH_PERIOD_MS, NULL);

  spectrum_init();
  lv_timer_create(spectrum_cb, SPECTRUM_PERIOD_MS, NULL);

  stats_cb(NULL);
  lv_timer_create(stats_cb, STATS_PERIOD_MS, NULL);
}

