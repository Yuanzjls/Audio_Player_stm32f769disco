/**
 * @file  lv_fs_sd.c
 * @brief LVGL "S:" drive on the SD card (read-only) with FatFs fast seek.
 *
 * TinyTTF reads glyph data from all over a font file (cmap, loca, hmtx,
 * glyf). Without fast seek, every backwards f_lseek walks the file's cluster
 * chain in the FAT from the beginning, costing extra SD reads per glyph. With
 * a cluster link map (FF_USE_FASTSEEK) a seek costs no I/O at all.
 *
 * A file stays open for as long as LVGL uses it (the font for good). When the
 * card is taken out and put back, its FatFs handle is no longer valid: a read
 * or seek that fails reopens the file by its path, at the same position, so
 * the font works again once the card is back.
 */
#include "lv_fs_sd.h"

#include <string.h>

#include "lvgl.h"

#define READ_CACHE_SIZE   4096U   /* LVGL's per-file read cache */
#define LINKMAP_INITIAL   64U     /* DWORDs; grown to what the file needs */

typedef struct
{
  FIL    fil;
  DWORD *linkmap;
  char  *path;                    /* to reopen the file after a remount */
} sd_file_t;

static lv_fs_drv_t s_drv;

bool fatfs_enable_fast_seek(FIL *file, DWORD *table, UINT table_len)
{
  table[0]    = table_len;
  file->cltbl = table;
  if (f_lseek(file, CREATE_LINKMAP) != FR_OK)
  {
    file->cltbl = NULL;
    return false;
  }
  return true;
}

/* Cluster link map: on FR_NOT_ENOUGH_CORE FatFs stores the size it needs in [0] */
static void setup_fast_seek(sd_file_t *f)
{
  lv_free(f->linkmap);
  f->linkmap = lv_malloc(LINKMAP_INITIAL * sizeof(DWORD));
  if (f->linkmap != NULL && !fatfs_enable_fast_seek(&f->fil, f->linkmap, LINKMAP_INITIAL))
  {
    const UINT needed = f->linkmap[0];
    lv_free(f->linkmap);
    f->linkmap = (needed > LINKMAP_INITIAL) ? lv_malloc(needed * sizeof(DWORD)) : NULL;
    if (f->linkmap != NULL && !fatfs_enable_fast_seek(&f->fil, f->linkmap, needed))
    {
      lv_free(f->linkmap);
      f->linkmap = NULL;
    }
  }
}

/* Open the file again at @p pos (after the card was taken out and put back) */
static bool reopen(sd_file_t *f, FSIZE_t pos)
{
  (void)f_close(&f->fil);
  if (f_open(&f->fil, f->path, FA_READ) != FR_OK)
  {
    return false;
  }
  setup_fast_seek(f);
  return f_lseek(&f->fil, pos) == FR_OK;
}

static void *fs_open(lv_fs_drv_t *drv, const char *path, lv_fs_mode_t mode)
{
  (void)drv;
  if (mode != LV_FS_MODE_RD)
  {
    return NULL;   /* the card is read-only */
  }

  sd_file_t *f = lv_malloc(sizeof(*f));
  const size_t path_len = strlen(path) + 1U;
  char *path_copy = lv_malloc(path_len);
  if (f == NULL || path_copy == NULL)
  {
    lv_free(f);
    lv_free(path_copy);
    return NULL;
  }
  if (f_open(&f->fil, path, FA_READ) != FR_OK)
  {
    lv_free(f);
    lv_free(path_copy);
    return NULL;
  }
  memcpy(path_copy, path, path_len);
  f->path    = path_copy;
  f->linkmap = NULL;
  setup_fast_seek(f);
  return f;
}

static lv_fs_res_t fs_close(lv_fs_drv_t *drv, void *file_p)
{
  (void)drv;
  sd_file_t *f = file_p;
  f_close(&f->fil);
  lv_free(f->linkmap);
  lv_free(f->path);
  lv_free(f);
  return LV_FS_RES_OK;
}

static lv_fs_res_t fs_read(lv_fs_drv_t *drv, void *file_p, void *buf, uint32_t btr, uint32_t *br)
{
  (void)drv;
  sd_file_t *f = file_p;
  const FSIZE_t pos = f_tell(&f->fil);
  UINT got = 0;
  FRESULT res = f_read(&f->fil, buf, btr, &got);
  if (res != FR_OK && reopen(f, pos))
  {
    res = f_read(&f->fil, buf, btr, &got);
  }
  *br = got;
  return (res == FR_OK) ? LV_FS_RES_OK : LV_FS_RES_UNKNOWN;
}

static lv_fs_res_t fs_seek(lv_fs_drv_t *drv, void *file_p, uint32_t pos, lv_fs_whence_t whence)
{
  (void)drv;
  sd_file_t *f = file_p;
  FSIZE_t target;

  switch (whence)
  {
    case LV_FS_SEEK_CUR: target = f_tell(&f->fil) + pos; break;
    case LV_FS_SEEK_END: target = f_size(&f->fil) + pos; break;
    default:             target = pos;                   break;
  }
  if (f_lseek(&f->fil, target) == FR_OK || reopen(f, target))
  {
    return LV_FS_RES_OK;
  }
  return LV_FS_RES_UNKNOWN;
}

static lv_fs_res_t fs_tell(lv_fs_drv_t *drv, void *file_p, uint32_t *pos_p)
{
  (void)drv;
  *pos_p = (uint32_t)f_tell(&((sd_file_t *)file_p)->fil);
  return LV_FS_RES_OK;
}

void lv_fs_sd_init(void)
{
  lv_fs_drv_init(&s_drv);
  s_drv.letter     = 'S';
  s_drv.cache_size = READ_CACHE_SIZE;
  s_drv.open_cb    = fs_open;
  s_drv.close_cb   = fs_close;
  s_drv.read_cb    = fs_read;
  s_drv.seek_cb    = fs_seek;
  s_drv.tell_cb    = fs_tell;
  lv_fs_drv_register(&s_drv);
}
