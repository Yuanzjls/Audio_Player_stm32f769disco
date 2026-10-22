/**
 * @file  shell.c
 * @brief Home screen with the app tiles and a now-playing bar; switching
 *        between it and the apps; fonts and the CPU load meter they share.
 *
 *   +----------------------------------------------------------+
 *   |  Music Player                                    v2.0.0  |
 *   |                                                          |
 *   |   +-------+  +-------+  +-------+  +-------+  +-------+  |
 *   |   |   ♪   |  |   ▶   |  |   ▤   |  |   i   |  |   ⚙   |  |
 *   |   | Music |  | Video |  | Files |  | System|  |Setting|  |
 *   |   +-------+  +-------+  +-------+  +-------+  +-------+  |
 *   |                                                          |
 *   |  +----------------------------------------------------+  |
 *   |  | [cover] Title / Artist                (▶)  (⏭)    |  |
 *   |  +----------------------------------------------------+  |
 *   +----------------------------------------------------------+
 */
#include "shell.h"

#include <stdio.h>
#include <string.h>

#include "display.h"
#include "log.h"
#include "main.h"
#include "player.h"
#include "settings.h"
#include "sysmon.h"
#include "theme.h"

#define STATUS_PERIOD_MS   250
#define CPU_PERIOD_TICKS   4          /* CPU load over 4 x 250 ms */
#define SWITCH_ANIM_MS     150

#ifdef WITH_DOOM                      /* six tiles */
#define TILE_SIZE          120
#define TILE_GAP           10
#define TILE_ICON          72
#else
#define TILE_SIZE          136
#define TILE_GAP           20
#define TILE_ICON          80
#endif
#define TILE_Y             120
#define BAR_HEIGHT         88
#define BAR_MARGIN         32
#define BAR_COVER          64

/* Fonts loaded from the SD card */
#define TITLE_FONT_SIZE    32
#define TEXT_FONT_SIZE     22

static const app_t *const k_apps[] = {
  &app_music, &app_video, &app_files,
#ifdef WITH_DOOM
  &app_doom,
#endif
  &app_sysinfo, &app_settings,
};
#define APP_COUNT (sizeof(k_apps) / sizeof(k_apps[0]))

static const char *const k_font_paths[] = { "S:/font.ttf", "S:/font.otf", "S:/font.ttc" };

typedef struct
{
  lv_obj_t *bar;
  lv_obj_t *cover;
  lv_obj_t *cover_note;
  lv_obj_t *title;
  lv_obj_t *artist;
  lv_obj_t *btn_play_icon;
  lv_obj_t *btn_next;
  lv_obj_t *btn_play;
} now_playing_bar_t;

static lv_obj_t          *s_home;
static lv_obj_t          *s_screens[APP_COUNT];
static const app_t       *s_current;          /* NULL: home screen */
static now_playing_bar_t  s_bar;
static player_status_t    s_status;
static uint32_t           s_shown_track = UINT32_MAX;
static player_state_t     s_shown_state = (player_state_t)-1;
static const void        *s_shown_cover = (const void *)-1;
static uint32_t           s_cpu_load;
static bool               s_settings_applied;
static int32_t            s_seen_volume = -1;   /* player volume at the last check */
static uint32_t           s_ticks;

static bool       s_fonts_tried;
static lv_font_t *s_font_title;
static lv_font_t *s_font_text;

const lv_font_t *shell_font_title(void) { return s_font_title ? s_font_title : &lv_font_montserrat_28; }
const lv_font_t *shell_font_text(void)  { return s_font_text  ? s_font_text  : &lv_font_montserrat_20; }
uint32_t shell_cpu_load(void)           { return s_cpu_load; }

/* ---- helpers --------------------------------------------------------------- */

static lv_obj_t *make_label(lv_obj_t *parent, const lv_font_t *font, lv_color_t color)
{
  lv_obj_t *label = lv_label_create(parent);
  lv_obj_set_style_text_font(label, font, 0);
  lv_obj_set_style_text_color(label, color, 0);
  lv_label_set_text(label, "");
  return label;
}

/* One line high, so that LV_LABEL_LONG_MODE_DOTS shortens a long text with
 * "..." instead of wrapping it (call again on a font change) */
static void one_line(lv_obj_t *label)
{
  lv_obj_set_height(label, lv_font_get_line_height(lv_obj_get_style_text_font(label, LV_PART_MAIN)));
}

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

static lv_obj_t *make_screen(void)
{
  lv_obj_t *scr = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(scr, COLOR_BG, 0);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
  lv_obj_set_scrollable(scr, false);
  return scr;
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

/* ---- switching ------------------------------------------------------------- */

static int app_index(const app_t *app)
{
  for (size_t i = 0; i < APP_COUNT; i++)
  {
    if (k_apps[i] == app)
    {
      return (int)i;
    }
  }
  return -1;
}

static void leave_current(void)
{
  if (s_current != NULL && s_current->hide != NULL)
  {
    s_current->hide();
  }
}

void shell_open(const app_t *app)
{
  const int i = app_index(app);
  if (i < 0 || app == s_current)
  {
    return;
  }
  leave_current();
  s_current = app;
  if (app->show != NULL)
  {
    app->show();
  }
  lv_screen_load_anim(s_screens[i], LV_SCREEN_LOAD_ANIM_FADE_IN, SWITCH_ANIM_MS, 0, false);
}

void shell_go_home(void)
{
  if (s_current == NULL)
  {
    return;
  }
  leave_current();
  s_current = NULL;
  s_shown_cover = (const void *)-1;   /* the cover may have changed meanwhile */
  lv_screen_load_anim(s_home, LV_SCREEN_LOAD_ANIM_FADE_IN, SWITCH_ANIM_MS, 0, false);
}

static void on_home_clicked(lv_event_t *e) { (void)e; shell_go_home(); }

lv_obj_t *shell_home_button(lv_obj_t *parent)
{
  lv_obj_t *btn = make_box(parent, COLOR_SURFACE);
  lv_obj_set_size(btn, 44, 40);
  lv_obj_set_style_bg_opa(btn, LV_OPA_TRANSP, 0);
  lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, LV_STATE_PRESSED);
  lv_obj_set_style_radius(btn, 8, 0);
  lv_obj_set_clickable(btn, true);
  lv_obj_set_ext_click_area(btn, 10);
  lv_obj_add_event_cb(btn, on_home_clicked, LV_EVENT_CLICKED, NULL);
  lv_obj_t *icon = make_label(btn, &lv_font_montserrat_28, COLOR_TEXT_DIM);
  lv_label_set_text(icon, LV_SYMBOL_HOME);
  lv_obj_center(icon);
  return btn;
}

lv_obj_t *shell_app_header(lv_obj_t *screen, const char *title)
{
  lv_obj_t *home = shell_home_button(screen);
  lv_obj_set_pos(home, 18, 12);

  lv_obj_t *label = make_label(screen, &lv_font_montserrat_28, COLOR_TEXT);
  lv_label_set_text(label, title);
  lv_obj_set_pos(label, 76, 16);

  lv_obj_t *content = make_box(screen, COLOR_BG);
  lv_obj_set_size(content, 800, 480 - 64);
  lv_obj_set_pos(content, 0, 64);
  lv_obj_set_style_border_side(content, LV_BORDER_SIDE_TOP, 0);
  lv_obj_set_style_border_color(content, COLOR_DIVIDER, 0);
  lv_obj_set_style_border_width(content, 1, 0);
  return content;
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
    lv_font_t *title = lv_tiny_ttf_create_file(k_font_paths[i], TITLE_FONT_SIZE);
    lv_font_t *text  = lv_tiny_ttf_create_file(k_font_paths[i], TEXT_FONT_SIZE);
    if (title == NULL || text == NULL)
    {
      LOG_WARN("ui: %s is not a usable TrueType/OpenType font", k_font_paths[i]);
      continue;
    }
    /* Glyphs missing from the font (e.g. LVGL symbols) come from the built-in one */
    title->fallback = &lv_font_montserrat_28;
    text->fallback  = &lv_font_montserrat_20;
    s_font_title = title;
    s_font_text  = text;
    LOG_INFO("ui: font %s loaded", k_font_paths[i]);

    lv_obj_set_style_text_font(s_bar.title, s_font_text, 0);
    lv_obj_set_style_text_font(s_bar.artist, s_font_text, 0);
    one_line(s_bar.title);
    one_line(s_bar.artist);
    s_shown_track = UINT32_MAX;
    for (size_t a = 0; a < APP_COUNT; a++)
    {
      if (k_apps[a]->fonts_changed != NULL)
      {
        k_apps[a]->fonts_changed();
      }
    }
    return;
  }
  LOG_WARN("ui: no font.ttf/font.otf/font.ttc on the SD card, non-Latin titles will not render");
}

/* ---- home screen ----------------------------------------------------------- */

static void on_tile_clicked(lv_event_t *e)
{
  shell_open((const app_t *)lv_event_get_user_data(e));
}

static void build_tile(lv_obj_t *home, const app_t *app, int32_t x)
{
  lv_obj_t *tile = make_box(home, COLOR_PANEL);
  lv_obj_set_size(tile, TILE_SIZE, TILE_SIZE);
  lv_obj_set_pos(tile, x, TILE_Y);
  lv_obj_set_style_radius(tile, 24, 0);
  lv_obj_set_style_bg_color(tile, COLOR_SURFACE, LV_STATE_PRESSED);
  lv_obj_set_clickable(tile, true);
  lv_obj_add_event_cb(tile, on_tile_clicked, LV_EVENT_CLICKED, (void *)app);

  lv_obj_t *circle = make_box(tile, lv_color_hex(app->color));
  lv_obj_set_size(circle, TILE_ICON, TILE_ICON);
  lv_obj_align(circle, LV_ALIGN_TOP_MID, 0, 16);
  lv_obj_set_style_radius(circle, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_clickable(circle, false);

  lv_obj_t *icon = make_label(circle, &lv_font_montserrat_40, COLOR_ON_ACCENT);
  lv_label_set_text(icon, app->icon);
  lv_obj_center(icon);

  lv_obj_t *name = make_label(tile, &lv_font_montserrat_20, COLOR_TEXT);
  lv_label_set_text(name, app->name);
  lv_obj_align(name, LV_ALIGN_BOTTOM_MID, 0, -12);
}

static void on_bar_clicked(lv_event_t *e)   { (void)e; shell_open(&app_music); }
static void on_bar_play(lv_event_t *e)      { (void)e; player_toggle_play(); }
static void on_bar_next(lv_event_t *e)      { (void)e; player_next(); }

static lv_obj_t *make_round_button(lv_obj_t *parent, int32_t size, lv_color_t bg, lv_color_t fg,
                                   const char *symbol, lv_obj_t **icon_out)
{
  lv_obj_t *btn = lv_button_create(parent);
  lv_obj_set_size(btn, size, size);
  lv_obj_set_style_radius(btn, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_bg_color(btn, bg, 0);
  lv_obj_set_style_shadow_width(btn, 0, 0);
  lv_obj_set_ext_click_area(btn, 8);
  lv_obj_t *icon = make_label(btn, &lv_font_montserrat_20, fg);
  lv_label_set_text(icon, symbol);
  lv_obj_center(icon);
  if (icon_out != NULL)
  {
    *icon_out = icon;
  }
  return btn;
}

static void build_bar(lv_obj_t *home)
{
  lv_obj_t *bar = make_box(home, COLOR_PANEL);
  s_bar.bar = bar;
  lv_obj_set_size(bar, 800 - 2 * BAR_MARGIN, BAR_HEIGHT);
  lv_obj_align(bar, LV_ALIGN_BOTTOM_MID, 0, -BAR_MARGIN);
  lv_obj_set_style_radius(bar, 20, 0);
  lv_obj_set_style_bg_color(bar, COLOR_SURFACE, LV_STATE_PRESSED);
  lv_obj_set_clickable(bar, true);
  lv_obj_add_event_cb(bar, on_bar_clicked, LV_EVENT_CLICKED, NULL);

  /* Cover: the song's picture, scaled down, or a note tile */
  s_bar.cover_note = make_box(bar, COLOR_ACCENT);
  lv_obj_set_size(s_bar.cover_note, BAR_COVER, BAR_COVER);
  lv_obj_align(s_bar.cover_note, LV_ALIGN_LEFT_MID, 12, 0);
  lv_obj_set_style_radius(s_bar.cover_note, 12, 0);
  lv_obj_set_clickable(s_bar.cover_note, false);
  lv_obj_t *note = make_label(s_bar.cover_note, &lv_font_montserrat_28, COLOR_ON_ACCENT);
  lv_label_set_text(note, LV_SYMBOL_AUDIO);
  lv_obj_center(note);

  s_bar.cover = lv_image_create(bar);
  lv_obj_set_size(s_bar.cover, BAR_COVER, BAR_COVER);
  lv_obj_align(s_bar.cover, LV_ALIGN_LEFT_MID, 12, 0);
  lv_image_set_inner_align(s_bar.cover, LV_IMAGE_ALIGN_CENTER);
  lv_obj_set_hidden(s_bar.cover, true);

  /* Text between the cover and the buttons (play 60 + gap 14 + next 52 + margin 18) */
  const int32_t text_x = 12 + BAR_COVER + 16;
  const int32_t text_w = 800 - 2 * BAR_MARGIN - text_x - (60 + 14 + 52 + 18) - 16;
  s_bar.title = make_label(bar, shell_font_text(), COLOR_TEXT);
  lv_label_set_long_mode(s_bar.title, LV_LABEL_LONG_MODE_DOTS);
  lv_obj_set_width(s_bar.title, text_w);
  one_line(s_bar.title);
  lv_obj_set_pos(s_bar.title, text_x, 12);

  s_bar.artist = make_label(bar, shell_font_text(), COLOR_TEXT_DIM);
  lv_label_set_long_mode(s_bar.artist, LV_LABEL_LONG_MODE_DOTS);
  lv_obj_set_width(s_bar.artist, text_w);
  one_line(s_bar.artist);
  lv_obj_set_pos(s_bar.artist, text_x, 46);

  s_bar.btn_next = make_round_button(bar, 52, COLOR_SURFACE, COLOR_TEXT, LV_SYMBOL_NEXT, NULL);
  lv_obj_align(s_bar.btn_next, LV_ALIGN_RIGHT_MID, -18, 0);
  lv_obj_add_event_cb(s_bar.btn_next, on_bar_next, LV_EVENT_CLICKED, NULL);

  s_bar.btn_play = make_round_button(bar, 60, COLOR_ACCENT, COLOR_ON_ACCENT, LV_SYMBOL_PLAY,
                                     &s_bar.btn_play_icon);
  lv_obj_align_to(s_bar.btn_play, s_bar.btn_next, LV_ALIGN_OUT_LEFT_MID, -14, 0);
  lv_obj_add_event_cb(s_bar.btn_play, on_bar_play, LV_EVENT_CLICKED, NULL);
}

static void build_home(void)
{
  s_home = make_screen();

  lv_obj_t *title = make_label(s_home, &lv_font_montserrat_28, COLOR_TEXT);
  lv_label_set_text(title, "Music Player");
  lv_obj_set_pos(title, 40, 28);

  lv_obj_t *version = make_label(s_home, &lv_font_montserrat_14, COLOR_TEXT_DIM);
  lv_label_set_text(version, firmware_version());
  lv_obj_align(version, LV_ALIGN_TOP_RIGHT, -40, 36);

  const int32_t row_w = (int32_t)APP_COUNT * TILE_SIZE + ((int32_t)APP_COUNT - 1) * TILE_GAP;
  for (size_t i = 0; i < APP_COUNT; i++)
  {
    build_tile(s_home, k_apps[i], (800 - row_w) / 2 + (int32_t)i * (TILE_SIZE + TILE_GAP));
  }
  build_bar(s_home);
}

/* Bring the now-playing bar up to date (only what changed: every change redraws) */
static void refresh_bar(void)
{
  const player_status_t *st = &s_status;
  const bool has_tracks = (st->state == PLAYER_STATE_STOPPED || st->state == PLAYER_STATE_PLAYING ||
                           st->state == PLAYER_STATE_PAUSED);

  if (!has_tracks)
  {
    if (s_shown_track != UINT32_MAX - 1U)
    {
      s_shown_track = UINT32_MAX - 1U;
      lv_label_set_text(s_bar.title, st->message[0] != '\0' ? st->message : "No music");
      lv_label_set_text(s_bar.artist, "");
    }
  }
  else if (st->track_serial != s_shown_track)
  {
    char split_artist[PLAYER_TITLE_MAX];
    const char *artist = st->artist;
    const char *title  = st->title;
    s_shown_track = st->track_serial;
    if (artist[0] == '\0')
    {
      /* No tags: an "Artist - Title" file name */
      split_name(st->title, split_artist, sizeof(split_artist), &title);
      artist = split_artist;
    }
    lv_label_set_text(s_bar.title, title);
    lv_label_set_text(s_bar.artist, artist[0] != '\0' ? artist : "Unknown artist");
  }

  if (st->state != s_shown_state)
  {
    s_shown_state = st->state;
    lv_label_set_text(s_bar.btn_play_icon, st->state == PLAYER_STATE_PLAYING ? LV_SYMBOL_PAUSE : LV_SYMBOL_PLAY);
    lv_obj_set_state(s_bar.btn_play, LV_STATE_DISABLED, !has_tracks);
    lv_obj_set_state(s_bar.btn_next, LV_STATE_DISABLED, !has_tracks);
  }

  const lv_image_dsc_t *cover = app_music_cover();
  if ((const void *)cover != s_shown_cover)
  {
    s_shown_cover = cover;
    if (cover != NULL)
    {
      lv_image_set_src(s_bar.cover, cover);
      lv_image_set_scale(s_bar.cover, (uint32_t)(LV_SCALE_NONE * BAR_COVER / cover->header.w));
    }
    lv_obj_set_hidden(s_bar.cover, cover == NULL);
    lv_obj_set_hidden(s_bar.cover_note, cover != NULL);
  }
}

/* Apply the loaded settings once, keep the saved volume in step with the player,
 * save changes, and switch the screen off when it has not been touched for long */
static void manage_settings(void)
{
  if (!settings_loaded())
  {
    return;
  }
  settings_t set = settings_get();
  if (!s_settings_applied)
  {
    s_settings_applied = true;
    display_set_brightness(set.brightness);
  }

  /* Volume changes (from any slider) are saved; only changes, so that the
   * volume of the player before it applied the settings is not taken */
  if (s_seen_volume >= 0 && s_status.volume != (uint8_t)s_seen_volume && s_status.volume != set.volume)
  {
    set.volume = s_status.volume;
    settings_set(&set);
  }
  s_seen_volume = s_status.volume;
  settings_save_if_due(lv_tick_get());

  if (set.screen_off_s != 0U && display_is_on() &&
      lv_display_get_inactive_time(NULL) >= (uint32_t)set.screen_off_s * 1000U)
  {
    display_set_on(false);
  }
}

static void status_cb(lv_timer_t *timer)
{
  (void)timer;
  player_get_status(&s_status);
  manage_settings();
  if (s_status.state != PLAYER_STATE_NO_CARD)
  {
    try_load_fonts();
  }
  if (++s_ticks % CPU_PERIOD_TICKS == 0U)
  {
    s_cpu_load = sysmon_cpu_load();
  }
  if (s_current == NULL)
  {
    refresh_bar();
  }
}

void shell_init(void)
{
  lv_obj_t *boot_screen = lv_screen_active();

  build_home();
  for (size_t i = 0; i < APP_COUNT; i++)
  {
    s_screens[i] = make_screen();
    k_apps[i]->create(s_screens[i]);
  }

  status_cb(NULL);
  lv_timer_create(status_cb, STATUS_PERIOD_MS, NULL);


  lv_screen_load(s_home);
  lv_obj_delete(boot_screen);
}
