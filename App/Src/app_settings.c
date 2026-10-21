/**
 * @file  app_settings.c
 * @brief Settings app: volume, screen brightness, shuffle at start, screen
 *        timeout. Changes apply at once and are saved on the SD card
 *        (settings.c, by the shell a moment after the last change).
 */
#include <stdio.h>

#include "display.h"
#include "player.h"
#include "settings.h"
#include "shell.h"
#include "theme.h"

#define PAD            40
#define ROW_H          84
#define LABEL_W        260
#define SLIDER_W       360
#define TOUCH_MARGIN   26

static const char *const k_timeout_names[] = { "Never", "1 min", "5 min", "10 min", "30 min", "" };
static const uint16_t    k_timeout_s[]     = { 0, 60, 300, 600, 1800 };
#define TIMEOUT_COUNT (sizeof(k_timeout_s) / sizeof(k_timeout_s[0]))

static lv_obj_t *s_volume;
static lv_obj_t *s_volume_value;
static lv_obj_t *s_brightness;
static lv_obj_t *s_brightness_value;
static lv_obj_t *s_shuffle;
static lv_obj_t *s_timeout;
static int32_t   s_last_volume_sent = -1;

static lv_obj_t *make_label(lv_obj_t *parent, const lv_font_t *font, lv_color_t color, const char *text)
{
  lv_obj_t *label = lv_label_create(parent);
  lv_obj_set_style_text_font(label, font, 0);
  lv_obj_set_style_text_color(label, color, 0);
  lv_label_set_text(label, text);
  return label;
}

/* Name (and a hint below it) on the left of row @p row */
static void row_title(lv_obj_t *parent, int32_t row, const char *name, const char *hint)
{
  const int32_t y = 24 + row * ROW_H;
  lv_obj_t *label = make_label(parent, &lv_font_montserrat_20, COLOR_TEXT, name);
  lv_obj_set_pos(label, PAD, y + (hint ? 4 : 14));
  if (hint != NULL)
  {
    /* Wrapped within the name column, clear of the control on the right */
    lv_obj_t *h = make_label(parent, &lv_font_montserrat_14, COLOR_TEXT_DIM, hint);
    lv_label_set_long_mode(h, LV_LABEL_LONG_MODE_WRAP);
    lv_obj_set_width(h, LABEL_W - 24);
    lv_obj_set_pos(h, PAD, y + 32);
  }
}

static lv_obj_t *make_slider(lv_obj_t *parent, int32_t row, int32_t min, int32_t max, lv_obj_t **value_out)
{
  lv_obj_t *slider = lv_slider_create(parent);
  lv_obj_set_size(slider, SLIDER_W, 14);
  lv_obj_set_pos(slider, PAD + LABEL_W, 24 + row * ROW_H + 18);
  lv_slider_set_range(slider, min, max);
  lv_obj_set_ext_click_area(slider, TOUCH_MARGIN);
  lv_obj_set_style_bg_color(slider, COLOR_SURFACE, LV_PART_MAIN);
  lv_obj_set_style_bg_color(slider, COLOR_ACCENT, LV_PART_INDICATOR);
  lv_obj_set_style_bg_color(slider, COLOR_ON_ACCENT, LV_PART_KNOB);
  lv_obj_set_style_border_color(slider, COLOR_ACCENT, LV_PART_KNOB);
  lv_obj_set_style_border_width(slider, 3, LV_PART_KNOB);
  lv_obj_set_style_pad_all(slider, 8, LV_PART_KNOB);

  *value_out = make_label(parent, &lv_font_montserrat_20, COLOR_TEXT_DIM, "");
  lv_obj_set_pos(*value_out, PAD + LABEL_W + SLIDER_W + 28, 24 + row * ROW_H + 12);
  return slider;
}

/* ---- events ---------------------------------------------------------------- */

static void on_volume(lv_event_t *e)
{
  const int32_t v = lv_slider_get_value(lv_event_get_target_obj(e));
  lv_label_set_text_fmt(s_volume_value, "%" LV_PRId32, v);
  if (v != s_last_volume_sent)
  {
    s_last_volume_sent = v;
    player_set_volume((uint8_t)v);    /* the shell saves it */
  }
}

static void on_brightness(lv_event_t *e)
{
  const int32_t v = lv_slider_get_value(lv_event_get_target_obj(e));
  lv_label_set_text_fmt(s_brightness_value, "%" LV_PRId32 " %%", v);
  settings_t s = settings_get();
  if (s.brightness != (uint8_t)v)
  {
    s.brightness = (uint8_t)v;
    settings_set(&s);
    display_set_brightness((uint8_t)v);
  }
}

static void on_shuffle(lv_event_t *e)
{
  settings_t s = settings_get();
  s.shuffle_on_start = lv_obj_has_state(lv_event_get_target_obj(e), LV_STATE_CHECKED);
  settings_set(&s);
}

static void on_timeout(lv_event_t *e)
{
  const uint32_t i = lv_buttonmatrix_get_selected_button(lv_event_get_target_obj(e));
  if (i < TIMEOUT_COUNT)
  {
    settings_t s = settings_get();
    s.screen_off_s = k_timeout_s[i];
    settings_set(&s);
  }
}

/* ---- app ------------------------------------------------------------------- */

static void settings_show(void)
{
  const settings_t s = settings_get();
  player_status_t st;
  player_get_status(&st);

  lv_slider_set_value(s_volume, st.volume, LV_ANIM_OFF);
  lv_label_set_text_fmt(s_volume_value, "%u", (unsigned)st.volume);
  s_last_volume_sent = st.volume;

  lv_slider_set_value(s_brightness, s.brightness, LV_ANIM_OFF);
  lv_label_set_text_fmt(s_brightness_value, "%u %%", (unsigned)s.brightness);

  lv_obj_set_state(s_shuffle, LV_STATE_CHECKED, s.shuffle_on_start);

  uint32_t selected = 0;
  for (uint32_t i = 0; i < TIMEOUT_COUNT; i++)
  {
    if (k_timeout_s[i] == s.screen_off_s)
    {
      selected = i;
    }
  }
  lv_buttonmatrix_set_button_ctrl(s_timeout, selected, LV_BUTTONMATRIX_CTRL_CHECKED);
}

static void settings_create(lv_obj_t *screen)
{
  lv_obj_t *c = shell_app_header(screen, "Settings");

  row_title(c, 0, "Volume", NULL);
  s_volume = make_slider(c, 0, 0, 100, &s_volume_value);
  lv_obj_add_event_cb(s_volume, on_volume, LV_EVENT_VALUE_CHANGED, NULL);

  row_title(c, 1, "Brightness", NULL);
  s_brightness = make_slider(c, 1, 10, 100, &s_brightness_value);
  lv_obj_add_event_cb(s_brightness, on_brightness, LV_EVENT_VALUE_CHANGED, NULL);

  row_title(c, 2, "Shuffle at start", "Start with shuffle on, at a random song");
  s_shuffle = lv_switch_create(c);
  lv_obj_set_size(s_shuffle, 64, 34);
  lv_obj_set_pos(s_shuffle, PAD + LABEL_W, 24 + 2 * ROW_H + 8);
  lv_obj_set_ext_click_area(s_shuffle, 12);
  lv_obj_set_style_bg_color(s_shuffle, COLOR_ACCENT, LV_PART_INDICATOR | LV_STATE_CHECKED);
  lv_obj_add_event_cb(s_shuffle, on_shuffle, LV_EVENT_VALUE_CHANGED, NULL);

  row_title(c, 3, "Screen off", "After this long without a touch");
  s_timeout = lv_buttonmatrix_create(c);
  lv_buttonmatrix_set_map(s_timeout, k_timeout_names);
  lv_buttonmatrix_set_button_ctrl_all(s_timeout, LV_BUTTONMATRIX_CTRL_CHECKABLE);
  lv_buttonmatrix_set_one_checked(s_timeout, true);
  lv_obj_set_size(s_timeout, 800 - 2 * PAD - LABEL_W, 60);
  lv_obj_set_pos(s_timeout, PAD + LABEL_W, 24 + 3 * ROW_H);
  lv_obj_set_style_pad_all(s_timeout, 4, 0);
  lv_obj_set_style_text_font(s_timeout, &lv_font_montserrat_14, 0);
  lv_obj_set_style_bg_color(s_timeout, COLOR_ACCENT, LV_PART_ITEMS | LV_STATE_CHECKED);
  lv_obj_set_style_text_color(s_timeout, COLOR_ON_ACCENT, LV_PART_ITEMS | LV_STATE_CHECKED);
  lv_obj_add_event_cb(s_timeout, on_timeout, LV_EVENT_VALUE_CHANGED, NULL);

  lv_obj_t *note = make_label(c, &lv_font_montserrat_14, COLOR_TEXT_DIM,
                              "Settings are saved on the SD card, in " SETTINGS_FILE ".");
  lv_obj_set_pos(note, PAD, 24 + 4 * ROW_H + 16);
}

const app_t app_settings = {
  .name   = "Settings",
  .icon   = LV_SYMBOL_SETTINGS,
  .color  = 0x5F6875,
  .create = settings_create,
  .show   = settings_show,
};
