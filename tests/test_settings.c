/**
 * @file  test_settings.c
 * @brief Settings file: defaults, parsing (hand-edited files, bad values,
 *        unknown keys), and saving only after the settings stopped changing.
 */
#include <string.h>

#include "check.h"
#include "fakes.h"
#include "settings.h"

static void put_file(const char *text)
{
  fakefs_add(SETTINGS_FILE, text, strlen(text));
}

/* The settings file as text ("" if there is none) */
static const char *file_text(void)
{
  static char text[1024];
  const uint8_t *data;
  size_t size;
  text[0] = '\0';
  if (fakefs_get(SETTINGS_FILE, &data, &size) && size < sizeof(text))
  {
    memcpy(text, data, size);
    text[size] = '\0';
  }
  return text;
}

static void test_defaults_without_file(void)
{
  fakefs_reset();
  settings_load();
  CHECK(settings_loaded());
  const settings_t s = settings_get();
  CHECK_EQ_INT(s.volume, 20);
  CHECK_EQ_INT(s.brightness, 80);
  CHECK(s.shuffle_on_start);
  CHECK_EQ_INT(s.screen_off_s, 0);
}

static void test_parse(void)
{
  fakefs_reset();
  put_file("volume=55\r\nbrightness=30\nshuffle_on_start=0\r\nscreen_off_s=300\r\n");
  settings_load();
  const settings_t s = settings_get();
  CHECK_EQ_INT(s.volume, 55);
  CHECK_EQ_INT(s.brightness, 30);
  CHECK(!s.shuffle_on_start);
  CHECK_EQ_INT(s.screen_off_s, 300);
}

/* Hand-edited files: bad or out-of-range values keep the default, unknown keys
 * and junk are ignored, the last line needs no line end */
static void test_bad_values_and_junk(void)
{
  fakefs_reset();
  put_file("# my settings\nvolume=150\nbrightness=abc\ncolour=red\n\n=5\nvolume\nscreen_off_s=60");
  settings_load();
  const settings_t s = settings_get();
  CHECK_EQ_INT(s.volume, 20);           /* 150 is out of range */
  CHECK_EQ_INT(s.brightness, 80);       /* not a number */
  CHECK_EQ_INT(s.screen_off_s, 60);     /* last line, no newline */

  /* Brightness has a floor so the screen never goes black by accident */
  put_file("brightness=5\n");
  settings_load();
  CHECK_EQ_INT(settings_get().brightness, 80);
}

/* A change is saved once the settings stayed unchanged for 2 s */
static void test_save_after_quiet_time(void)
{
  fakefs_reset();
  settings_load();
  settings_t s = settings_get();

  s.volume = 70;
  settings_set(&s);
  settings_save_if_due(1000);                   /* starts the delay */
  settings_save_if_due(2500);                   /* 1.5 s: not yet */
  CHECK_EQ_STR(file_text(), "");

  s.volume = 71;                                /* another change restarts it */
  settings_set(&s);
  settings_save_if_due(2600);
  settings_save_if_due(4000);
  CHECK_EQ_STR(file_text(), "");
  settings_save_if_due(4700);                   /* 2.1 s after the last change */
  CHECK(strstr(file_text(), "volume=71\r\n") != NULL);

  /* What was saved loads back */
  s.brightness = 45;
  s.shuffle_on_start = false;
  s.screen_off_s = 600;
  settings_set(&s);
  settings_save_if_due(5000);
  settings_save_if_due(8000);
  settings_load();
  const settings_t back = settings_get();
  CHECK_EQ_INT(back.volume, 71);
  CHECK_EQ_INT(back.brightness, 45);
  CHECK(!back.shuffle_on_start);
  CHECK_EQ_INT(back.screen_off_s, 600);
}

/* Setting the same values again writes nothing */
static void test_no_write_without_change(void)
{
  fakefs_reset();
  put_file("volume=33\n");
  settings_load();
  settings_t s = settings_get();
  settings_set(&s);
  settings_save_if_due(1000);
  settings_save_if_due(9000);
  CHECK_EQ_STR(file_text(), "volume=33\n");
}

int main(void)
{
  RUN(test_defaults_without_file);
  RUN(test_parse);
  RUN(test_bad_values_and_junk);
  RUN(test_save_after_quiet_time);
  RUN(test_no_write_without_change);
  fakefs_reset();
  return CHECK_RESULT();
}
