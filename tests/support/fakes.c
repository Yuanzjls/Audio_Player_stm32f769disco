/**
 * @file  fakes.c
 * @brief See fakes.h. The FatFs functions are the subset the decoders and the
 *        ID3 reader use, with FatFs semantics (f_tell / f_size are macros on
 *        the FIL fields, so those fields are kept up to date).
 */
#include "fakes.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "audio_out.h"
#include "ff.h"
#include "log.h"
#include "lv_fs_sd.h"

#define MAX_FILES 16

typedef struct
{
  char    path[256];
  uint8_t *data;
  size_t   size;
} fake_file_t;

static fake_file_t s_files[MAX_FILES];
static size_t      s_fail_after;     /* bytes until f_read() fails, 0 = never */
static bool        s_fail_armed;

void fakefs_reset(void)
{
  for (size_t i = 0; i < MAX_FILES; i++)
  {
    free(s_files[i].data);
    memset(&s_files[i], 0, sizeof(s_files[i]));
  }
  s_fail_armed = false;
}

void fakefs_add(const char *path, const void *data, size_t size)
{
  for (size_t i = 0; i < MAX_FILES; i++)
  {
    if (s_files[i].path[0] == '\0' || strcmp(s_files[i].path, path) == 0)
    {
      free(s_files[i].data);
      snprintf(s_files[i].path, sizeof(s_files[i].path), "%s", path);
      s_files[i].data = malloc(size ? size : 1U);
      memcpy(s_files[i].data, data, size);
      s_files[i].size = size;
      return;
    }
  }
  fprintf(stderr, "fakefs: too many files\n");
  abort();
}

bool fakefs_get(const char *path, const uint8_t **data, size_t *size)
{
  for (size_t i = 0; i < MAX_FILES; i++)
  {
    if (s_files[i].path[0] != '\0' && strcmp(s_files[i].path, path) == 0)
    {
      *data = s_files[i].data;
      *size = s_files[i].size;
      return true;
    }
  }
  return false;
}

bool fakefs_add_host_file(const char *path, const char *host_path)
{
  FILE *f = fopen(host_path, "rb");
  if (f == NULL)
  {
    fprintf(stderr, "fakefs: cannot open %s\n", host_path);
    return false;
  }
  fseek(f, 0, SEEK_END);
  const long size = ftell(f);
  fseek(f, 0, SEEK_SET);
  uint8_t *buf = malloc((size_t)size);
  const bool ok = fread(buf, 1, (size_t)size, f) == (size_t)size;
  fclose(f);
  if (ok)
  {
    fakefs_add(path, buf, (size_t)size);
  }
  free(buf);
  return ok;
}

void fakefs_fail_reads_after(size_t bytes)
{
  s_fail_after = bytes;
  s_fail_armed = (bytes != 0U);
}

/* The open file is remembered in the object's (otherwise unused) FATFS pointer */
static fake_file_t *file_of(FIL *fp)
{
  return (fake_file_t *)(void *)fp->obj.fs;
}

FRESULT f_open(FIL *fp, const TCHAR *path, BYTE mode)
{
  memset(fp, 0, sizeof(*fp));
  if ((mode & FA_CREATE_ALWAYS) != 0U)
  {
    fakefs_add(path, "", 0);            /* new, empty file */
  }
  for (size_t i = 0; i < MAX_FILES; i++)
  {
    if (s_files[i].path[0] != '\0' && strcmp(s_files[i].path, path) == 0)
    {
      fp->obj.fs      = (FATFS *)(void *)&s_files[i];
      fp->obj.objsize = s_files[i].size;
      fp->fptr        = 0;
      return FR_OK;
    }
  }
  return FR_NO_FILE;
}

FRESULT f_close(FIL *fp)
{
  fp->obj.fs = NULL;
  return FR_OK;
}

FRESULT f_read(FIL *fp, void *buff, UINT btr, UINT *br)
{
  fake_file_t *f = file_of(fp);
  *br = 0;
  if (f == NULL)
  {
    return FR_INVALID_OBJECT;
  }
  if (s_fail_armed)
  {
    if (btr >= s_fail_after)
    {
      s_fail_armed = false;
      return FR_DISK_ERR;
    }
    s_fail_after -= btr;
  }
  size_t n = (fp->fptr < f->size) ? f->size - (size_t)fp->fptr : 0U;
  if (n > btr)
  {
    n = btr;
  }
  memcpy(buff, f->data + fp->fptr, n);
  fp->fptr += n;
  *br = (UINT)n;
  return FR_OK;
}

FRESULT f_write(FIL *fp, const void *buff, UINT btw, UINT *bw)
{
  fake_file_t *f = file_of(fp);
  *bw = 0;
  if (f == NULL)
  {
    return FR_INVALID_OBJECT;
  }
  const size_t end = (size_t)fp->fptr + btw;
  if (end > f->size)
  {
    f->data = realloc(f->data, end);
    f->size = end;
    fp->obj.objsize = end;
  }
  memcpy(f->data + fp->fptr, buff, btw);
  fp->fptr += btw;
  *bw = btw;
  return FR_OK;
}

FRESULT f_lseek(FIL *fp, FSIZE_t ofs)
{
  fake_file_t *f = file_of(fp);
  if (f == NULL)
  {
    return FR_INVALID_OBJECT;
  }
  /* In read mode FatFs clips the position to the file size */
  fp->fptr = (ofs > f->size) ? f->size : ofs;
  return FR_OK;
}

bool fatfs_enable_fast_seek(FIL *file, DWORD *table, UINT table_len)
{
  (void)file;
  (void)table;
  (void)table_len;
  return false;   /* fast seek is only an optimisation */
}

/* ---- log ---- */

static unsigned s_warnings;
static unsigned s_errors;

void fakelog_reset(void)       { s_warnings = s_errors = 0; }
unsigned fakelog_warnings(void) { return s_warnings; }
unsigned fakelog_errors(void)   { return s_errors; }

void log_printf(const char *level, const char *fmt, ...)
{
  if (level[0] == 'W')
  {
    s_warnings++;
  }
  else if (level[0] == 'E')
  {
    s_errors++;
  }
  if (getenv("TEST_LOG") != NULL)
  {
    va_list ap;
    va_start(ap, fmt);
    fprintf(stderr, "    [%s] ", level);
    vfprintf(stderr, fmt, ap);
    fputc('\n', stderr);
    va_end(ap);
  }
}

/* ---- audio output ---- */

static bool     s_audio_on;
static float    s_freq;
static float    s_ampl;
static uint32_t s_rate;
static uint64_t s_phase;   /* samples generated so far */

void fakeaudio_sine(float frequency_hz, float amplitude, uint32_t sample_rate)
{
  s_audio_on = true;
  s_freq     = frequency_hz;
  s_ampl     = amplitude;
  s_rate     = sample_rate;
}

void fakeaudio_stop(void)
{
  s_audio_on = false;
}

bool audio_out_peek(int16_t *out, uint32_t frames)
{
  if (!s_audio_on)
  {
    return false;
  }
  for (uint32_t i = 0; i < frames; i++, s_phase++)
  {
    const double t = (double)s_phase / (double)s_rate;
    const int16_t v = (int16_t)lrint(32767.0 * s_ampl * sin(2.0 * 3.14159265358979 * s_freq * t));
    out[2U * i]      = v;
    out[2U * i + 1U] = v;
  }
  return true;
}
