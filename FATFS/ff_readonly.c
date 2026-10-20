/*-----------------------------------------------------------------------*/
/* Write stub for the read-only FatFs configuration (FF_FS_READONLY = 1) */
/*-----------------------------------------------------------------------*/
/* The SD card is only ever read. LVGL's FatFs file-system driver still   */
/* references f_write(), so provide one that refuses every write.         */
/*-----------------------------------------------------------------------*/

#include <stddef.h>

#include "ff.h"

#if FF_FS_READONLY

FRESULT f_write (FIL* fp, const void* buff, UINT btw, UINT* bw)
{
	(void)fp;
	(void)buff;
	(void)btw;
	if (bw != NULL)
	{
		*bw = 0;
	}
	return FR_DENIED;
}

#endif
