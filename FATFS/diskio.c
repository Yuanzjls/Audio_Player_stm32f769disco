/*-----------------------------------------------------------------------*/
/* Low level disk I/O module for FatFs     (C)ChaN, 2016                 */
/*-----------------------------------------------------------------------*/
/* Glue between FatFs and the STM32F769I-Discovery microSD card (BSP SD  */
/* driver, DMA transfers synchronised with a FreeRTOS semaphore).        */
/* Only physical drive 0 (the SD card) is supported (FF_VOLUMES = 1).    */
/*-----------------------------------------------------------------------*/

#include "ff.h"			/* Obtains integer types */
#include "diskio.h"		/* Declarations of disk functions */
#include "stm32f769i_discovery_sd.h"
#include "semphr.h"

/* Physical drive number of the SD card */
#define DEV_MMC		0

#define SD_TIMEOUT 1000
#define SD_DEFAULT_BLOCK_SIZE 512

/* Given from the SD DMA completion callbacks below */
static SemaphoreHandle_t xSemaphore;

/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber to identify the drive */
)
{
	DSTATUS stat;

	if (pdrv != DEV_MMC) return STA_NOINIT;

	stat = STA_NOINIT;
	if(BSP_SD_GetCardState() == MSD_OK)
	{
		stat &= ~STA_NOINIT;
	}
	return stat;
}



/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
	BYTE pdrv				/* Physical drive nmuber to identify the drive */
)
{
	DSTATUS stat;

	if (pdrv != DEV_MMC) return STA_NOINIT;

	stat = BSP_SD_Init();
	xSemaphore = xSemaphoreCreateBinary();
	return stat;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber to identify the drive */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Start sector in LBA */
	UINT count		/* Number of sectors to read */
)
{
	DRESULT res;
	uint32_t timer;

	if (pdrv != DEV_MMC) return RES_PARERR;

	res = RES_ERROR;

	if(BSP_SD_ReadBlocks_DMA((uint32_t*)buff,
	(uint32_t) (sector),
	count) == MSD_OK)
	{
		/* Wait for BSP_SD_ReadCpltCallback() */
		if( xSemaphoreTake( xSemaphore, ( TickType_t ) SD_TIMEOUT ) == pdTRUE )
		{
			timer = xTaskGetTickCount() + SD_TIMEOUT;
			while(timer > xTaskGetTickCount())
			{
				if (BSP_SD_GetCardState()== MSD_OK)
				{
					res = RES_OK;
					break;
				}
			}
		}
	}

	return res;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if FF_FS_READONLY == 0

DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber to identify the drive */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Start sector in LBA */
	UINT count			/* Number of sectors to write */
)
{
	DRESULT res;

	if (pdrv != DEV_MMC) return RES_PARERR;

	res = RES_ERROR;

	if(BSP_SD_WriteBlocks_DMA((uint32_t*)buff,
	(uint32_t)(sector),
	count) == MSD_OK)
	{
		/* Wait for BSP_SD_WriteCpltCallback() */
		if( xSemaphoreTake( xSemaphore, ( TickType_t ) SD_TIMEOUT ) == pdTRUE )
		{
			while(BSP_SD_GetCardState()!= MSD_OK)
			{
				;
			}
			res = RES_OK;
		}
	}

	return res;
}

#endif


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
	DRESULT res;
	HAL_SD_CardInfoTypeDef CardInfo;

	if (pdrv != DEV_MMC) return RES_PARERR;

	BSP_SD_GetCardInfo(&CardInfo);
	switch (cmd)
	{
	/* Make sure that no pending write process */
	case CTRL_SYNC :
		res = RES_OK;
		break;

	/* Get number of sectors on the disk (DWORD) */
	case GET_SECTOR_COUNT :
		*(DWORD*)buff = CardInfo.LogBlockNbr;
		res = RES_OK;
		break;

	/* Get R/W sector size (WORD) */
	case GET_SECTOR_SIZE :
		*(WORD*)buff = CardInfo.LogBlockSize;
		res = RES_OK;
		break;

	/* Get erase block size in unit of sector (DWORD) */
	case GET_BLOCK_SIZE :
		*(DWORD*)buff = CardInfo.LogBlockSize / SD_DEFAULT_BLOCK_SIZE;
		res = RES_OK;
		break;

	default:
		res = RES_PARERR;
	}

	return res;
}

/* No RTC-backed timestamps (the file system is mounted read-only) */
DWORD get_fattime(void)
{
	return 0;
}

/**
  * @brief BSP Tx Transfer completed callbacks
  * @retval None
  */
void BSP_SD_WriteCpltCallback(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	/* Wake up disk_write() */
	xSemaphoreGiveFromISR( xSemaphore, &xHigherPriorityTaskWoken );

	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

/**
  * @brief BSP Rx Transfer completed callbacks
  * @retval None
  */
void BSP_SD_ReadCpltCallback(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	/* Wake up disk_read() */
	xSemaphoreGiveFromISR( xSemaphore, &xHigherPriorityTaskWoken );

	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}
