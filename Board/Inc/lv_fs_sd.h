/**
 * @file  lv_fs_sd.h
 * @brief Read-only LVGL file system driver ("S:") for the SD card, with
 *        FatFs fast seek so random access in large files (fonts) is cheap.
 */
#ifndef LV_FS_SD_H
#define LV_FS_SD_H

#include <stdbool.h>

#include "ff.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Register the "S:" drive with LVGL. Call after lv_init(). */
void lv_fs_sd_init(void);

/**
 * Enable FatFs fast seek on an open read-only file: builds the cluster map in
 * @p table (@p table_len DWORDs). Returns false (normal seeking stays in use)
 * if the file is too fragmented for the table.
 */
bool fatfs_enable_fast_seek(FIL *file, DWORD *table, UINT table_len);

#ifdef __cplusplus
}
#endif

#endif /* LV_FS_SD_H */
