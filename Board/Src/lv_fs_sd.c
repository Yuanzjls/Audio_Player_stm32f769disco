/**
 * @file  lv_fs_sd.c
 * @brief LVGL "S:" drive on the SD card (read-only) with FatFs fast seek.
 *
 * TinyTTF reads glyph data from all over a font file (cmap, loca, hmtx,
 * glyf). Without fast seek, every backwards f_lseek walks the file's cluster
 * chain in the FAT from the beginning, costing extra SD reads per glyph. With
 * a cluster link map (FF_USE_FASTSEEK) a seek costs no I/O at all.
 */
#include "lv_fs_sd.h"

#include "lvgl.h"

#define READ_CACHE_SIZE   4096U   /* LVGL's per-file read cache */
#define LINKMAP_INITIAL   64U     /* DWORDs; grown to what the file needs */

typedef struct
{
  FIL    fil;
  DWORD *linkmap;
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

static void *fs_open(lv_fs_drv_t *drv, const char *path, lv_fs_mode_t mode)
{
  (void)drv;
  if (mode != LV_FS_MODE_RD)
  {
    return NULL;   /* the card is read-only */
  }

  sd_file_t *f = lv_malloc(sizeof(*f));
  if (f == NULL)
  {
    return NULL;
  }
  if (f_open(&f->fil, path, FA_READ) != FR_OK)
  {
    lv_free(f);
    return NULL;
  }

  /* Cluster link map: on FR_NOT_ENOUGH_CORE FatFs stores the size it needs in [0] */
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
  return f;
}

static lv_fs_res_t fs_close(lv_fs_drv_t *drv, void *file_p)
{
  (void)drv;
  sd_file_t *f = file_p;
  f_close(&f->fil);
  lv_free(f->linkmap);
  lv_free(f);
  return LV_FS_RES_OK;
}

static lv_fs_res_t fs_read(lv_fs_drv_t *drv, void *file_p, void *buf, uint32_t btr, uint32_t *br)
{
  (void)drv;
  UINT got = 0;
  const FRESULT res = f_read(&((sd_file_t *)file_p)->fil, buf, btr, &got);
  *br = got;
  return (res == FR_OK) ? LV_FS_RES_OK : LV_FS_RES_UNKNOWN;
}

static lv_fs_res_t fs_seek(lv_fs_drv_t *drv, void *file_p, uint32_t pos, lv_fs_whence_t whence)
{
  (void)drv;
  FIL *fil = &((sd_file_t *)file_p)->fil;
  FSIZE_t target;

  switch (whence)
  {
    case LV_FS_SEEK_CUR: target = f_tell(fil) + pos; break;
    case LV_FS_SEEK_END: target = f_size(fil) + pos; break;
    default:             target = pos;               break;
  }
  return (f_lseek(fil, target) == FR_OK) ? LV_FS_RES_OK : LV_FS_RES_UNKNOWN;
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
