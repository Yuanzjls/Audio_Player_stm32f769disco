/**
 * @file  settings.c
 * @brief User settings in /settings.txt, e.g.
 *
 *     volume=40
 *     brightness=80
 *     shuffle_on_start=1
 *     screen_off_s=300
 *
 * Unknown keys and bad values are ignored (the default stays), so the file can
 * be edited by hand and older or newer firmware reads it too.
 */
#include "settings.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ff.h"
#include "log.h"

#define FILE_MAX      512U
#define SAVE_DELAY_MS 2000U     /* unchanged this long before the file is written */

static const settings_t k_defaults = {
  .volume           = 20,
  .brightness       = 80,
  .shuffle_on_start = true,
  .screen_off_s     = 0,
};

static settings_t    s_settings = k_defaults;
static volatile bool s_loaded;
static bool          s_dirty;
static uint32_t      s_changed_ms;

/* "key=value": the value if the key matches and it lies in [min, max] */
static bool parse_value(const char *line, const char *key, long min, long max, long *value)
{
  const size_t n = strlen(key);
  if (strncmp(line, key, n) != 0 || line[n] != '=')
  {
    return false;
  }
  char *end;
  const long v = strtol(&line[n + 1U], &end, 10);
  if (end == &line[n + 1U] || v < min || v > max)
  {
    return false;
  }
  *value = v;
  return true;
}

/* Line by line (not strtok(): newlib-nano allocates its state on first use,
 * and aborts when that fails) */
static void parse(char *text, settings_t *s)
{
  char *next = text;
  while (*next != '\0')
  {
    char *line = next;
    const size_t len = strcspn(line, "\r\n");
    next = line + len;
    if (*next != '\0')
    {
      *next++ = '\0';
    }
    long v;
    if (parse_value(line, "volume", 0, 100, &v))
    {
      s->volume = (uint8_t)v;
    }
    else if (parse_value(line, "brightness", 10, 100, &v))
    {
      s->brightness = (uint8_t)v;
    }
    else if (parse_value(line, "shuffle_on_start", 0, 1, &v))
    {
      s->shuffle_on_start = (v != 0);
    }
    else if (parse_value(line, "screen_off_s", 0, 3600, &v))
    {
      s->screen_off_s = (uint16_t)v;
    }
  }
}

void settings_load(void)
{
  FIL file;
  settings_t s = k_defaults;

  if (f_open(&file, SETTINGS_FILE, FA_READ) == FR_OK)
  {
    char text[FILE_MAX + 1U];
    UINT got = 0;
    if (f_read(&file, text, FILE_MAX, &got) == FR_OK)
    {
      text[got] = '\0';
      parse(text, &s);
    }
    f_close(&file);
    LOG_INFO("settings: loaded");
  }
  s_settings = s;
  s_loaded   = true;
}

bool settings_loaded(void)
{
  return s_loaded;
}

settings_t settings_get(void)
{
  return s_settings;
}

void settings_set(const settings_t *settings)
{
  if (memcmp(settings, &s_settings, sizeof(s_settings)) == 0)
  {
    return;
  }
  s_settings   = *settings;
  s_dirty      = true;
  s_changed_ms = 0;          /* restarts the delay at the next settings_save_if_due() */
}

void settings_save_if_due(uint32_t now_ms)
{
  if (!s_dirty || !s_loaded)
  {
    return;
  }
  if (s_changed_ms == 0U)
  {
    s_changed_ms = now_ms | 1U;
    return;
  }
  if (now_ms - s_changed_ms < SAVE_DELAY_MS)
  {
    return;
  }
  s_dirty = false;

  char text[FILE_MAX];
  const int len = snprintf(text, sizeof(text),
                           "volume=%u\r\nbrightness=%u\r\nshuffle_on_start=%u\r\nscreen_off_s=%u\r\n",
                           (unsigned)s_settings.volume, (unsigned)s_settings.brightness,
                           s_settings.shuffle_on_start ? 1U : 0U, (unsigned)s_settings.screen_off_s);
  FIL file;
  UINT written = 0;
  FRESULT res = f_open(&file, SETTINGS_FILE, FA_WRITE | FA_CREATE_ALWAYS);
  if (res == FR_OK)
  {
    res = f_write(&file, text, (UINT)len, &written);
    const FRESULT close_res = f_close(&file);
    res = (res != FR_OK) ? res : close_res;
  }
  if (res != FR_OK || written != (UINT)len)
  {
    LOG_WARN("settings: cannot write %s (%d)", SETTINGS_FILE, (int)res);
  }
}
