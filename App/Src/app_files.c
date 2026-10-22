/**
 * @file  app_files.c
 * @brief Files app: browse the folders of the SD card.
 *
 * Folders come first, then files, each sorted by name (hidden and system
 * entries are left out). A tap on a folder opens it; on a song of /Music it
 * plays the song in the Music app; on an .avi file it plays the video in the
 * Video app; on any other file it shows its details and, for text files, the
 * beginning of the text.
 *
 * The list is virtualized like the music library: a pool of row widgets is
 * re-labelled while scrolling, so only visible names are measured and drawn.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "decoder.h"
#include "ff.h"
#include "player.h"
#include "shell.h"
#include "theme.h"

#define MAX_ENTRIES   2048
#define PATH_MAX_LEN  512
#define BAR_H         56          /* path bar under the app header */
#define ROW_H         56
#define LIST_H        (480 - 64 - BAR_H - 1)
#define POOL          ((LIST_H / ROW_H) + 3)
#define NAME_X        72
#define INFO_W        150         /* size and date column on the right */
#define TOAST_MS      2500
#define TEXT_MAX      8192        /* bytes of a text file the viewer shows */

typedef struct
{
  char     name[FF_LFN_BUF + 1];  /* UTF-8 */
  uint32_t size;
  uint16_t date;                  /* FAT date and time */
  uint16_t time;
  uint8_t  attr;
} entry_t;

typedef enum { KIND_DIR, KIND_AUDIO, KIND_VIDEO, KIND_IMAGE, KIND_TEXT, KIND_OTHER } kind_t;

/* Large: in SDRAM */
static entry_t  s_entries[MAX_ENTRIES] __attribute__((section(".sdram")));
static uint16_t s_order[MAX_ENTRIES] __attribute__((section(".sdram")));
static char     s_text[TEXT_MAX + 1] __attribute__((section(".sdram")));
static uint32_t s_count;
static bool     s_truncated;          /* the folder has more than MAX_ENTRIES entries */
static char     s_path[PATH_MAX_LEN] = "/";

static lv_obj_t   *s_up;
static lv_obj_t   *s_path_label;
static lv_obj_t   *s_summary;
static lv_obj_t   *s_list;
static lv_obj_t   *s_spacer;
static lv_obj_t   *s_empty;
static lv_obj_t   *s_rows[POOL];
static int32_t     s_row_index[POOL];  /* entry shown by each row, -1 = none */
static lv_obj_t   *s_toast;
static lv_timer_t *s_toast_timer;

/* Viewer: file details and the beginning of a text file, over the whole screen */
static lv_obj_t *s_viewer;
static lv_obj_t *s_viewer_title;
static lv_obj_t *s_viewer_details;
static lv_obj_t *s_viewer_body;
static lv_obj_t *s_viewer_text;

/* ---- helpers --------------------------------------------------------------- */

static lv_obj_t *make_label(lv_obj_t *parent, const lv_font_t *font, lv_color_t color)
{
  lv_obj_t *label = lv_label_create(parent);
  lv_obj_set_style_text_font(label, font, 0);
  lv_obj_set_style_text_color(label, color, 0);
  lv_label_set_text(label, "");
  return label;
}

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

static lv_obj_t *make_icon_button(lv_obj_t *parent, const char *symbol, lv_event_cb_t cb)
{
  lv_obj_t *btn = make_box(parent, COLOR_SURFACE);
  lv_obj_set_size(btn, 44, 40);
  lv_obj_set_style_bg_opa(btn, LV_OPA_TRANSP, 0);
  lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, LV_STATE_PRESSED);
  lv_obj_set_style_radius(btn, 8, 0);
  lv_obj_set_style_text_opa(btn, LV_OPA_30, LV_STATE_DISABLED);
  lv_obj_set_clickable(btn, true);
  lv_obj_set_ext_click_area(btn, 10);
  lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, NULL);
  lv_obj_t *icon = make_label(btn, &lv_font_montserrat_28, COLOR_TEXT_DIM);
  lv_label_set_text(icon, symbol);
  lv_obj_center(icon);
  return btn;
}

static bool has_ext(const char *name, const char *const *exts)
{
  const char *dot = strrchr(name, '.');
  if (dot == NULL)
  {
    return false;
  }
  for (; *exts != NULL; exts++)
  {
    if (strcasecmp(dot + 1, *exts) == 0)
    {
      return true;
    }
  }
  return false;
}

static kind_t kind_of(const entry_t *e)
{
  static const char *const images[] = { "jpg", "jpeg", "png", "bmp", "gif", NULL };
  static const char *const texts[]  = { "txt", "log", "ini", "cfg", "conf", "json", "csv", "md", "lrc",
                                        "xml", "htm", "html", NULL };
  static const char *const videos[] = { "avi", NULL };

  if ((e->attr & AM_DIR) != 0U)    return KIND_DIR;
  if (decoder_is_supported(e->name)) return KIND_AUDIO;
  if (has_ext(e->name, videos))    return KIND_VIDEO;
  if (has_ext(e->name, images))    return KIND_IMAGE;
  if (has_ext(e->name, texts))     return KIND_TEXT;
  return KIND_OTHER;
}

static void format_size(char *buf, size_t len, uint32_t bytes)
{
  if (bytes < 1024U)
  {
    snprintf(buf, len, "%lu B", (unsigned long)bytes);
  }
  else if (bytes < 1024U * 1024U)
  {
    snprintf(buf, len, "%lu.%lu KB", (unsigned long)(bytes / 1024U), (unsigned long)(bytes % 1024U * 10U / 1024U));
  }
  else if (bytes < 1024U * 1024U * 1024U)
  {
    const uint32_t kb = bytes / 1024U;
    snprintf(buf, len, "%lu.%lu MB", (unsigned long)(kb / 1024U), (unsigned long)(kb % 1024U * 10U / 1024U));
  }
  else
  {
    const uint32_t mb = bytes / (1024U * 1024U);
    snprintf(buf, len, "%lu.%lu GB", (unsigned long)(mb / 1024U), (unsigned long)(mb % 1024U * 10U / 1024U));
  }
}

static void format_date(char *buf, size_t len, uint16_t date, uint16_t time)
{
  snprintf(buf, len, "%04u-%02u-%02u %02u:%02u", 1980U + (date >> 9), (date >> 5) & 15U, date & 31U,
           (unsigned)(time >> 11), (time >> 5) & 63U);
}

static bool at_root(void)
{
  return strcmp(s_path, "/") == 0;
}

/* @p path + "/" + @p name into @p out; false if it does not fit */
static bool join_path(char *out, size_t len, const char *path, const char *name)
{
  const int n = snprintf(out, len, "%s%s%s", path, (strcmp(path, "/") == 0) ? "" : "/", name);
  return n > 0 && (size_t)n < len;
}

static void on_toast_timer(lv_timer_t *timer)
{
  lv_obj_set_hidden(s_toast, true);
  lv_timer_pause(timer);
}

static void toast(const char *text)
{
  lv_label_set_text(s_toast, text);
  lv_obj_set_hidden(s_toast, false);
  lv_obj_move_foreground(s_toast);
  lv_timer_reset(s_toast_timer);
  lv_timer_resume(s_toast_timer);
}

/* ---- folder ---------------------------------------------------------------- */

static int compare_entries(const void *a, const void *b)
{
  const entry_t *ea = &s_entries[*(const uint16_t *)a];
  const entry_t *eb = &s_entries[*(const uint16_t *)b];
  const bool da = (ea->attr & AM_DIR) != 0U;
  const bool db = (eb->attr & AM_DIR) != 0U;
  if (da != db)
  {
    return da ? -1 : 1;
  }
  return strcasecmp(ea->name, eb->name);
}

/* Read the folder s_path into s_entries / s_order */
static FRESULT scan(void)
{
  DIR dir;
  FILINFO fi;
  s_count = 0;
  s_truncated = false;

  FRESULT res = f_opendir(&dir, s_path);
  if (res != FR_OK)
  {
    return res;
  }
  while ((res = f_readdir(&dir, &fi)) == FR_OK && fi.fname[0] != '\0')
  {
    if ((fi.fattrib & (AM_HID | AM_SYS)) != 0U)
    {
      continue;
    }
    if (s_count == MAX_ENTRIES)
    {
      s_truncated = true;
      break;
    }
    entry_t *e = &s_entries[s_count];
    snprintf(e->name, sizeof(e->name), "%s", fi.fname);
    e->size = (uint32_t)fi.fsize;
    e->date = fi.fdate;
    e->time = fi.ftime;
    e->attr = fi.fattrib;
    s_order[s_count] = (uint16_t)s_count;
    s_count++;
  }
  f_closedir(&dir);
  qsort(s_order, s_count, sizeof(s_order[0]), compare_entries);
  return res;
}

/* Free space of the card, once per call of the app (FatFs keeps the count after
 * the first time, which may scan the whole FAT) */
static void show_summary(void)
{
  char text[96];
  int n = snprintf(text, sizeof(text), "%lu%s %s", (unsigned long)s_count, s_truncated ? "+" : "",
                   s_count == 1U ? "item" : "items");
  DWORD free_clusters;
  FATFS *fs;
  if (f_getfree("", &free_clusters, &fs) == FR_OK && n > 0 && (size_t)n < sizeof(text))
  {
    const uint32_t free_mb  = (uint32_t)(((uint64_t)free_clusters * fs->csize * FF_MAX_SS) >> 20);
    const uint32_t total_mb = (uint32_t)(((uint64_t)(fs->n_fatent - 2U) * fs->csize * FF_MAX_SS) >> 20);
    snprintf(&text[n], sizeof(text) - (size_t)n, ", %lu.%lu of %lu.%lu GB free", (unsigned long)(free_mb / 1024U),
             (unsigned long)(free_mb % 1024U * 10U / 1024U), (unsigned long)(total_mb / 1024U),
             (unsigned long)(total_mb % 1024U * 10U / 1024U));
  }
  lv_label_set_text(s_summary, text);
}

/* The path, its beginning cut off if it is long */
static void show_path(void)
{
  const size_t max = 56;
  const char *p = s_path;
  const size_t n = strlen(p);
  if (n > max)
  {
    const char *slash = strchr(&p[n - max], '/');
    lv_label_set_text_fmt(s_path_label, "...%s", slash != NULL ? slash : &p[n - max]);
  }
  else
  {
    lv_label_set_text(s_path_label, p);
  }
  lv_obj_set_state(s_up, LV_STATE_DISABLED, at_root());
}

static const char *kind_symbol(kind_t kind)
{
  switch (kind)
  {
    case KIND_DIR:   return LV_SYMBOL_DIRECTORY;
    case KIND_AUDIO: return LV_SYMBOL_AUDIO;
    case KIND_VIDEO: return LV_SYMBOL_VIDEO;
    case KIND_IMAGE: return LV_SYMBOL_IMAGE;
    default:         return LV_SYMBOL_FILE;
  }
}

static lv_color_t kind_color(kind_t kind)
{
  switch (kind)
  {
    case KIND_DIR:   return lv_color_hex(0xF9AB00);
    case KIND_AUDIO: return COLOR_ACCENT;
    case KIND_VIDEO: return lv_color_hex(0xE8590C);
    case KIND_IMAGE: return lv_color_hex(0x0F9D58);
    case KIND_TEXT:  return COLOR_TEXT;
    default:         return COLOR_TEXT_DIM;
  }
}

/* Row children: 0 icon, 1 name, 2 size, 3 date */
static void bind_row(uint32_t slot, int32_t index)
{
  lv_obj_t *row = s_rows[slot];
  if (index < 0 || index >= (int32_t)s_count)
  {
    lv_obj_set_hidden(row, true);
    s_row_index[slot] = -1;
    return;
  }
  if (s_row_index[slot] != index)
  {
    const entry_t *e = &s_entries[s_order[index]];
    const kind_t kind = kind_of(e);
    char text[24];

    lv_obj_t *icon = lv_obj_get_child(row, 0);
    lv_label_set_text(icon, kind_symbol(kind));
    lv_obj_set_style_text_color(icon, kind_color(kind), 0);
    lv_label_set_text(lv_obj_get_child(row, 1), e->name);
    if (kind == KIND_DIR)
    {
      lv_label_set_text(lv_obj_get_child(row, 2), LV_SYMBOL_RIGHT);
    }
    else
    {
      format_size(text, sizeof(text), e->size);
      lv_label_set_text(lv_obj_get_child(row, 2), text);
    }
    format_date(text, sizeof(text), e->date, e->time);
    lv_label_set_text(lv_obj_get_child(row, 3), text);
    lv_obj_set_y(row, index * ROW_H);
    s_row_index[slot] = index;
  }
  lv_obj_set_hidden(row, false);
}

static void layout_rows(void)
{
  int32_t first = lv_obj_get_scroll_y(s_list) / ROW_H;
  if (first < 0)
  {
    first = 0;
  }
  for (int32_t i = first; i < first + (int32_t)POOL; i++)
  {
    bind_row((uint32_t)i % POOL, i);
  }
}

static void on_list_scroll(lv_event_t *e)
{
  (void)e;
  layout_rows();
}

/* Read s_path and list it, scrolled to the entry named @p select (NULL: the top) */
static void load(const char *select)
{
  FRESULT res = scan();
  if (res != FR_OK && !at_root())
  {
    /* Gone (card changed?): back to the root */
    strcpy(s_path, "/");
    select = NULL;
    res = scan();
  }
  show_path();

  for (uint32_t k = 0; k < POOL; k++)
  {
    s_row_index[k] = -1;
  }
  lv_obj_set_height(s_spacer, (int32_t)s_count * ROW_H);
  lv_obj_update_layout(s_list);

  int32_t y = 0;
  if (select != NULL)
  {
    for (uint32_t i = 0; i < s_count; i++)
    {
      if (strcmp(s_entries[s_order[i]].name, select) == 0)
      {
        y = (i > 2U) ? (int32_t)(i - 2U) * ROW_H : 0;
        break;
      }
    }
  }
  lv_obj_scroll_to_y(s_list, y, LV_ANIM_OFF);
  layout_rows();

  if (res != FR_OK)
  {
    player_status_t st;
    player_get_status(&st);
    lv_label_set_text(s_empty, st.state == PLAYER_STATE_NO_CARD ? "No SD card." : "The SD card cannot be read.");
    lv_label_set_text(s_summary, "");
  }
  else
  {
    lv_label_set_text(s_empty, "This folder is empty.");
    show_summary();
  }
  lv_obj_set_hidden(s_empty, res == FR_OK && s_count != 0U);
}

static void go_up(void)
{
  if (at_root())
  {
    return;
  }
  char child[FF_LFN_BUF + 1];
  char *slash = strrchr(s_path, '/');
  snprintf(child, sizeof(child), "%s", slash + 1);
  if (slash == s_path)
  {
    slash[1] = '\0';   /* "/name" -> "/" */
  }
  else
  {
    *slash = '\0';
  }
  load(child);
}

static void on_up_clicked(lv_event_t *e)
{
  (void)e;
  go_up();
}

/* ---- viewer ---------------------------------------------------------------- */

/* The first TEXT_MAX bytes of @p path into s_text, made displayable; false
 * if it does not look like text */
static bool read_text(const char *path, bool *ascii)
{
  FIL fil;
  UINT got = 0;
  if (f_open(&fil, path, FA_READ) != FR_OK)
  {
    return false;
  }
  const FRESULT res = f_read(&fil, s_text, TEXT_MAX, &got);
  f_close(&fil);
  if (res != FR_OK)
  {
    return false;
  }

  /* Skip a UTF-8 byte order mark; do not end in the middle of a character */
  uint32_t start = (got >= 3U && memcmp(s_text, "\xEF\xBB\xBF", 3) == 0) ? 3U : 0U;
  if (got == TEXT_MAX)
  {
    uint32_t cut = got;
    while (cut > start && ((uint8_t)s_text[cut - 1U] & 0xC0U) == 0x80U)
    {
      cut--;
    }
    if (cut > start && ((uint8_t)s_text[cut - 1U] & 0x80U) != 0U)
    {
      cut--;   /* the lead byte of the cut character */
    }
    got = cut;
  }

  /* Tabs to spaces, carriage returns out; NUL bytes mean binary */
  uint32_t out = 0;
  *ascii = true;
  for (uint32_t i = start; i < got; i++)
  {
    const char c = s_text[i];
    if (c == '\0')
    {
      return false;
    }
    if (c == '\r')
    {
      continue;
    }
    if (((uint8_t)c & 0x80U) != 0U)
    {
      *ascii = false;
    }
    s_text[out++] = (c == '\t') ? ' ' : c;
  }
  s_text[out] = '\0';
  return true;
}

static void open_viewer(const char *path, const entry_t *e, kind_t kind)
{
  char size[24];
  char date[24];
  format_size(size, sizeof(size), e->size);
  format_date(date, sizeof(date), e->date, e->time);
  lv_label_set_text(s_viewer_title, e->name);
  lv_label_set_text_fmt(s_viewer_details, "%s, %s%s", size, date, (e->attr & AM_RDO) ? ", read-only" : "");

  bool ascii = true;
  const char *message = NULL;
  if (kind == KIND_TEXT || kind == KIND_OTHER)
  {
    if (!read_text(path, &ascii))
    {
      message = "No preview: this is not a text file.";
    }
  }
  else if (kind == KIND_AUDIO)
  {
    message = "Songs are played from " PLAYER_MUSIC_DIR ": copy it there to play it.";
  }
  else
  {
    message = "No preview for this type of file.";
  }

  if (message != NULL)
  {
    lv_obj_set_style_text_font(s_viewer_text, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(s_viewer_text, COLOR_TEXT_DIM, 0);
    lv_label_set_text(s_viewer_text, message);
  }
  else
  {
    /* The SD-card font is slow on first use of each glyph: only when needed */
    lv_obj_set_style_text_font(s_viewer_text, ascii ? &lv_font_montserrat_20 : shell_font_text(), 0);
    lv_obj_set_style_text_color(s_viewer_text, COLOR_TEXT, 0);
    lv_label_set_text_static(s_viewer_text, s_text[0] != '\0' ? s_text : " ");
    if (e->size > TEXT_MAX)
    {
      lv_label_set_text_fmt(s_viewer_details, "%s, %s, first %u KB shown", size, date, TEXT_MAX / 1024U);
    }
  }
  lv_obj_scroll_to_y(s_viewer_body, 0, LV_ANIM_OFF);
  lv_obj_set_hidden(s_viewer, false);
  lv_obj_move_foreground(s_viewer);
}

static void close_viewer(void)
{
  lv_obj_set_hidden(s_viewer, true);
  lv_label_set_text(s_viewer_text, "");   /* drop the reference to s_text */
}

static void on_viewer_back(lv_event_t *e)
{
  (void)e;
  close_viewer();
}

/* ---- opening --------------------------------------------------------------- */

/* Library index of the song at @p path, -1 if it is not in the library */
static int32_t library_index(const char *path)
{
  player_status_t st;
  player_get_status(&st);
  char track[PATH_MAX_LEN];
  for (uint16_t i = 0; i < st.track_count; i++)
  {
    if (player_get_track_path(i, track, sizeof(track)) && strcasecmp(track, path) == 0)
    {
      return i;
    }
  }
  return -1;
}

static void open_entry(int32_t index)
{
  const entry_t *e = &s_entries[s_order[index]];
  const kind_t kind = kind_of(e);
  char path[PATH_MAX_LEN];
  if (!join_path(path, sizeof(path), s_path, e->name))
  {
    toast("The path is too long.");
    return;
  }

  switch (kind)
  {
    case KIND_DIR:
      strcpy(s_path, path);
      load(NULL);
      return;

    case KIND_AUDIO:
    {
      const int32_t track = library_index(path);
      if (track >= 0)
      {
        player_play_track((uint16_t)track);
        shell_open(&app_music);
        return;
      }
      break;
    }

    case KIND_VIDEO:
      if (!app_video_play_file(path, &app_files))
      {
        toast("This video cannot be played (MJPEG .avi only).");
      }
      return;

    default:
      break;
  }
  open_viewer(path, e, kind);
}

static void on_row_clicked(lv_event_t *e)
{
  const uintptr_t slot = (uintptr_t)lv_event_get_user_data(e);
  if (s_row_index[slot] >= 0)
  {
    open_entry(s_row_index[slot]);
  }
}

/* ---- app ------------------------------------------------------------------- */

static void build_viewer(lv_obj_t *screen)
{
  s_viewer = make_box(screen, COLOR_BG);
  lv_obj_set_size(s_viewer, 800, 480);
  lv_obj_set_hidden(s_viewer, true);

  lv_obj_t *back = make_icon_button(s_viewer, LV_SYMBOL_LEFT, on_viewer_back);
  lv_obj_set_pos(back, 18, 12);

  s_viewer_title = make_label(s_viewer, shell_font_text(), COLOR_TEXT);
  lv_label_set_long_mode(s_viewer_title, LV_LABEL_LONG_MODE_DOTS);
  lv_obj_set_width(s_viewer_title, 800 - 76 - 24);
  one_line(s_viewer_title);
  lv_obj_set_pos(s_viewer_title, 76, 6);

  s_viewer_details = make_label(s_viewer, &lv_font_montserrat_14, COLOR_TEXT_DIM);
  lv_obj_set_pos(s_viewer_details, 76, 38);

  s_viewer_body = make_box(s_viewer, COLOR_PANEL);
  lv_obj_set_size(s_viewer_body, 800, 480 - 64);
  lv_obj_set_pos(s_viewer_body, 0, 64);
  lv_obj_set_style_border_side(s_viewer_body, LV_BORDER_SIDE_TOP, 0);
  lv_obj_set_style_border_color(s_viewer_body, COLOR_DIVIDER, 0);
  lv_obj_set_style_border_width(s_viewer_body, 1, 0);
  lv_obj_set_style_pad_hor(s_viewer_body, 32, 0);
  lv_obj_set_style_pad_ver(s_viewer_body, 20, 0);
  lv_obj_set_scrollable(s_viewer_body, true);
  lv_obj_set_scroll_dir(s_viewer_body, LV_DIR_VER);

  s_viewer_text = make_label(s_viewer_body, &lv_font_montserrat_20, COLOR_TEXT);
  lv_label_set_long_mode(s_viewer_text, LV_LABEL_LONG_MODE_WRAP);
  lv_obj_set_width(s_viewer_text, lv_pct(100));
}

static void files_create(lv_obj_t *screen)
{
  lv_obj_t *c = shell_app_header(screen, "Files");

  /* Path bar: up, path, item count and free space */
  s_up = make_icon_button(c, LV_SYMBOL_UP, on_up_clicked);
  lv_obj_set_pos(s_up, 18, 9);

  s_path_label = make_label(c, shell_font_text(), COLOR_TEXT);
  lv_label_set_long_mode(s_path_label, LV_LABEL_LONG_MODE_DOTS);
  lv_obj_set_width(s_path_label, 380);
  one_line(s_path_label);
  lv_obj_align(s_path_label, LV_ALIGN_TOP_LEFT, 76, (BAR_H - lv_obj_get_height(s_path_label)) / 2);

  s_summary = make_label(c, &lv_font_montserrat_14, COLOR_TEXT_DIM);
  lv_obj_align(s_summary, LV_ALIGN_TOP_RIGHT, -24, (BAR_H - 16) / 2);

  s_list = make_box(c, COLOR_PANEL);
  lv_obj_set_size(s_list, 800, LIST_H);
  lv_obj_set_pos(s_list, 0, BAR_H);
  lv_obj_set_scrollable(s_list, true);
  lv_obj_set_scroll_dir(s_list, LV_DIR_VER);
  lv_obj_set_style_border_side(s_list, LV_BORDER_SIDE_TOP, 0);
  lv_obj_set_style_border_color(s_list, COLOR_DIVIDER, 0);
  lv_obj_set_style_border_width(s_list, 1, 0);
  lv_obj_add_event_cb(s_list, on_list_scroll, LV_EVENT_SCROLL, NULL);

  /* Invisible, as tall as the whole folder: gives the list its scroll range */
  s_spacer = make_box(s_list, COLOR_PANEL);
  lv_obj_set_size(s_spacer, 1, 0);
  lv_obj_set_style_bg_opa(s_spacer, LV_OPA_TRANSP, 0);
  lv_obj_set_clickable(s_spacer, false);

  for (uint32_t k = 0; k < POOL; k++)
  {
    lv_obj_t *row = make_box(s_list, COLOR_PANEL);
    lv_obj_set_size(row, 800, ROW_H);
    lv_obj_set_clickable(row, true);
    lv_obj_set_style_bg_color(row, COLOR_ACCENT_SOFT, LV_STATE_PRESSED);
    lv_obj_set_style_border_side(row, LV_BORDER_SIDE_BOTTOM, 0);
    lv_obj_set_style_border_color(row, COLOR_DIVIDER, 0);
    lv_obj_set_style_border_width(row, 1, 0);
    lv_obj_add_event_cb(row, on_row_clicked, LV_EVENT_CLICKED, (void *)(uintptr_t)k);

    lv_obj_t *icon = make_label(row, &lv_font_montserrat_28, COLOR_TEXT_DIM);
    lv_obj_align(icon, LV_ALIGN_LEFT_MID, 24, 0);

    lv_obj_t *name = make_label(row, shell_font_text(), COLOR_TEXT);
    lv_label_set_long_mode(name, LV_LABEL_LONG_MODE_DOTS);
    lv_obj_set_width(name, 800 - NAME_X - INFO_W - 40);
    one_line(name);
    lv_obj_align(name, LV_ALIGN_LEFT_MID, NAME_X, 0);

    lv_obj_t *size = make_label(row, &lv_font_montserrat_20, COLOR_TEXT_DIM);
    lv_obj_set_width(size, INFO_W);
    lv_obj_set_style_text_align(size, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_align(size, LV_ALIGN_TOP_RIGHT, -24, 6);

    lv_obj_t *date = make_label(row, &lv_font_montserrat_14, COLOR_TEXT_DIM);
    lv_obj_set_width(date, INFO_W);
    lv_obj_set_style_text_align(date, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_align(date, LV_ALIGN_TOP_RIGHT, -24, 32);

    lv_obj_set_hidden(row, true);
    s_rows[k]      = row;
    s_row_index[k] = -1;
  }

  s_empty = make_label(c, &lv_font_montserrat_20, COLOR_TEXT_DIM);
  lv_obj_align(s_empty, LV_ALIGN_TOP_MID, 0, BAR_H + 48);
  lv_obj_set_hidden(s_empty, true);

  s_toast = make_label(screen, &lv_font_montserrat_20, COLOR_ON_ACCENT);
  lv_obj_set_style_bg_color(s_toast, COLOR_TEXT, 0);
  lv_obj_set_style_bg_opa(s_toast, LV_OPA_90, 0);
  lv_obj_set_style_radius(s_toast, 12, 0);
  lv_obj_set_style_pad_hor(s_toast, 20, 0);
  lv_obj_set_style_pad_ver(s_toast, 12, 0);
  lv_obj_align(s_toast, LV_ALIGN_BOTTOM_MID, 0, -24);
  lv_obj_set_hidden(s_toast, true);
  s_toast_timer = lv_timer_create(on_toast_timer, TOAST_MS, NULL);
  lv_timer_pause(s_toast_timer);

  build_viewer(screen);
}

static void files_show(void)
{
  /* Read again: the card may have changed. Keep the place in the folder. */
  const int32_t y = lv_obj_get_scroll_y(s_list);
  load(NULL);
  lv_obj_scroll_to_y(s_list, y, LV_ANIM_OFF);
  layout_rows();
}

static void files_hide(void)
{
  close_viewer();
  lv_obj_set_hidden(s_toast, true);
  lv_timer_pause(s_toast_timer);
}

/* The shell loaded the SD-card fonts: use them for the names */
static void files_fonts_changed(void)
{
  lv_obj_set_style_text_font(s_path_label, shell_font_text(), 0);
  one_line(s_path_label);
  lv_obj_set_y(s_path_label, (BAR_H - lv_obj_get_height(s_path_label)) / 2);
  lv_obj_set_style_text_font(s_viewer_title, shell_font_text(), 0);
  one_line(s_viewer_title);
  for (uint32_t k = 0; k < POOL; k++)
  {
    lv_obj_t *name = lv_obj_get_child(s_rows[k], 1);
    lv_obj_set_style_text_font(name, shell_font_text(), 0);
    one_line(name);
    lv_obj_align(name, LV_ALIGN_LEFT_MID, NAME_X, 0);
    s_row_index[k] = -1;
  }
  layout_rows();
}

const app_t app_files = {
  .name          = "Files",
  .icon          = LV_SYMBOL_DIRECTORY,
  .color         = 0xF9AB00,
  .create        = files_create,
  .show          = files_show,
  .hide          = files_hide,
  .fonts_changed = files_fonts_changed,
};
