/**
 * @file  doom_libc.c
 * @brief stdio, malloc and friends for the Doom engine (see doom_libc.h).
 */
#include "doom_libc.h"

#include <stdbool.h>

#include "doom_port.h"
#include "ff.h"
#include "log.h"
#include "src/stdlib/builtin/lv_tlsf.h"

#define HEAP_SIZE (512U * 1024U)       /* the engine's own allocations: lump directory (~120 KB for DOOM2.WAD), the current song's MIDI events (up to ~150 KB) */
#define CONSOLE_LINE  160

struct doom_file
{
  FIL  fil;
  bool eof;
};

/* ---- memory ---------------------------------------------------------------- */

static uint8_t    s_heap[HEAP_SIZE] __attribute__((section(".sdram"), aligned(8)));
static lv_tlsf_t  s_tlsf;

static lv_tlsf_t heap(void)
{
  if (s_tlsf == NULL)
  {
    s_tlsf = lv_tlsf_create_with_pool(s_heap, sizeof(s_heap));
  }
  return s_tlsf;
}

void *doom_malloc(size_t size)
{
  void *p = lv_tlsf_malloc(heap(), size != 0U ? size : 1U);
  if (p == NULL)
  {
    LOG_ERROR("doom: out of heap (%u bytes)", (unsigned)size);
  }
  return p;
}

void *doom_calloc(size_t count, size_t size)
{
  void *p = doom_malloc(count * size);
  if (p != NULL)
  {
    memset(p, 0, count * size);
  }
  return p;
}

void *doom_realloc(void *ptr, size_t size)
{
  if (ptr == NULL)
  {
    return doom_malloc(size);
  }
  return lv_tlsf_realloc(heap(), ptr, size);
}

void doom_free(void *ptr)
{
  if (ptr != NULL)
  {
    lv_tlsf_free(heap(), ptr);
  }
}

char *doom_strdup(const char *s)
{
  const size_t n = strlen(s) + 1U;
  char *d = doom_malloc(n);
  if (d != NULL)
  {
    memcpy(d, s, n);
  }
  return d;
}

/* ---- console: the log, a line at a time ------------------------------------ */

static char   s_line[CONSOLE_LINE];
static size_t s_line_len;

static void console_char(char c)
{
  if (c == '\n' || s_line_len == sizeof(s_line) - 1U)
  {
    s_line[s_line_len] = '\0';
    if (s_line_len != 0U)
    {
      LOG_INFO("doom: %s", s_line);
    }
    s_line_len = 0;
    if (c == '\n')
    {
      return;
    }
  }
  s_line[s_line_len++] = c;
}

static int console_write(const char *s, size_t n)
{
  for (size_t i = 0; i < n; i++)
  {
    console_char(s[i]);
  }
  return (int)n;
}

int doom_putchar(int c)
{
  console_char((char)c);
  return c;
}

int doom_puts(const char *s)
{
  console_write(s, strlen(s));
  console_char('\n');
  return 1;
}

int doom_printf(const char *fmt, ...)
{
  va_list ap;
  va_start(ap, fmt);
  const int n = doom_vfprintf(stdout, fmt, ap);
  va_end(ap);
  return n;
}

/* ---- files ----------------------------------------------------------------- */

static bool is_console(const doom_file_t *f)
{
  return f == stdout || f == stderr;
}

doom_file_t *doom_fopen(const char *path, const char *mode)
{
  BYTE fmode;
  if (mode[0] == 'r')
  {
    fmode = (strchr(mode, '+') != NULL) ? (FA_READ | FA_WRITE) : FA_READ;
  }
  else if (mode[0] == 'a')
  {
    fmode = FA_OPEN_APPEND | FA_WRITE;
  }
  else
  {
    fmode = FA_CREATE_ALWAYS | FA_WRITE;
  }
  doom_file_t *f = doom_malloc(sizeof(*f));
  if (f == NULL)
  {
    return NULL;
  }
  if (f_open(&f->fil, path, fmode) != FR_OK)
  {
    doom_free(f);
    return NULL;
  }
  f->eof = false;
  return f;
}

int doom_fclose(doom_file_t *f)
{
  if (f == NULL || is_console(f))
  {
    return 0;
  }
  const FRESULT res = f_close(&f->fil);
  doom_free(f);
  return (res == FR_OK) ? 0 : EOF;
}

size_t doom_fread(void *buf, size_t size, size_t count, doom_file_t *f)
{
  if (is_console(f) || size == 0U)
  {
    return 0;
  }
  UINT got = 0;
  const UINT want = (UINT)(size * count);
  if (f_read(&f->fil, buf, want, &got) != FR_OK || got < want)
  {
    f->eof = true;
  }
  return got / size;
}

int doom_fgetc(doom_file_t *f)
{
  unsigned char c;
  return (doom_fread(&c, 1, 1, f) == 1U) ? c : EOF;
}

size_t doom_fwrite(const void *buf, size_t size, size_t count, doom_file_t *f)
{
  if (size == 0U)
  {
    return 0;
  }
  if (is_console(f))
  {
    return (size_t)console_write(buf, size * count) / size;
  }
  UINT put = 0;
  (void)f_write(&f->fil, buf, (UINT)(size * count), &put);
  return put / size;
}

int doom_fseek(doom_file_t *f, long offset, int whence)
{
  if (is_console(f))
  {
    return -1;
  }
  FSIZE_t base = 0;
  if (whence == SEEK_CUR)
  {
    base = f_tell(&f->fil);
  }
  else if (whence == SEEK_END)
  {
    base = f_size(&f->fil);
  }
  f->eof = false;
  return (f_lseek(&f->fil, (FSIZE_t)((long)base + offset)) == FR_OK) ? 0 : -1;
}

long doom_ftell(doom_file_t *f)
{
  return is_console(f) ? -1L : (long)f_tell(&f->fil);
}

int doom_feof(doom_file_t *f)
{
  return (!is_console(f) && (f->eof || f_eof(&f->fil))) ? 1 : 0;
}

int doom_fflush(doom_file_t *f)
{
  if (f != NULL && !is_console(f))
  {
    (void)f_sync(&f->fil);
  }
  return 0;
}

int doom_vfprintf(doom_file_t *f, const char *fmt, va_list ap)
{
  char buf[256];
  const int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  if (n <= 0)
  {
    return n;
  }
  const size_t len = ((size_t)n < sizeof(buf)) ? (size_t)n : sizeof(buf) - 1U;
  if (is_console(f))
  {
    return console_write(buf, len);
  }
  return (int)doom_fwrite(buf, 1, len, f);
}

int doom_fprintf(doom_file_t *f, const char *fmt, ...)
{
  va_list ap;
  va_start(ap, fmt);
  const int n = doom_vfprintf(f, fmt, ap);
  va_end(ap);
  return n;
}

/* The engine reads its configuration with fscanf(f, "%79s %[^\n]\n", ...):
 * one line at a time, then sscanf */
int doom_fscanf(doom_file_t *f, const char *fmt, ...)
{
  char line[CONSOLE_LINE];
  size_t n = 0;
  char c;
  UINT got;
  if (is_console(f))
  {
    return EOF;
  }
  while (n < sizeof(line) - 1U && f_read(&f->fil, &c, 1, &got) == FR_OK && got == 1U)
  {
    if (c == '\n')
    {
      break;
    }
    if (c != '\r')
    {
      line[n++] = c;
    }
  }
  if (n == 0U && f_eof(&f->fil))
  {
    f->eof = true;
    return EOF;
  }
  line[n] = '\0';
  va_list ap;
  va_start(ap, fmt);
  const int r = vsscanf(line, fmt, ap);
  va_end(ap);
  return r;
}

int doom_remove(const char *path)
{
  return (f_unlink(path) == FR_OK) ? 0 : -1;
}

int doom_mkdir(const char *path)
{
  /* The engine passes directories with a trailing separator */
  char dir[256];
  size_t n = strlen(path);
  if (n >= sizeof(dir))
  {
    return -1;
  }
  memcpy(dir, path, n + 1U);
  while (n > 1U && (dir[n - 1U] == '/' || dir[n - 1U] == '\\'))
  {
    dir[--n] = '\0';
  }
  const FRESULT res = f_mkdir(dir);
  return (res == FR_OK || res == FR_EXIST) ? 0 : -1;
}

/* ---- process --------------------------------------------------------------- */

char *doom_getenv(const char *name)
{
  (void)name;
  return NULL;
}

int doom_system(const char *command)
{
  (void)command;
  return -1;
}

void doom_exit(int status)
{
  doom_port_ended(status == 0 ? NULL : "Doom stopped.");
}
