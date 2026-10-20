/*-----------------------------------------------------------------------*/
/* Low level disk I/O module for FatFs     (C)ChaN, 2016                 */
/*-----------------------------------------------------------------------*/
/* Glue between FatFs and the STM32F769I-Discovery microSD card (BSP SD  */
/* driver, DMA transfers synchronised with a FreeRTOS semaphore).        */
/* Only physical drive 0 (the SD card) is supported (FF_VOLUMES = 1).    */
/*-----------------------------------------------------------------------*/

#include <string.h>

#include "ff.h"			/* Obtains integer types */
#include "diskio.h"		/* Declarations of disk functions */
#include "stm32f769i_discovery_sd.h"
#include "FreeRTOS.h"
#include "semphr.h"

/* Physical drive number of the SD card */
#define DEV_MMC		0

#define SD_TIMEOUT 1000
#define SD_DEFAULT_BLOCK_SIZE 512

/* Internal SRAM (DTCM + SRAM1/2) is never cached (see the MPU setup in main.c),
 * so the SD DMA can write there directly. Anything else (e.g. buffers in the
 * cacheable SDRAM) goes through this bounce buffer. */
#define DMA_SAFE_START	0x20000000UL
#define DMA_SAFE_END	0x20080000UL
#define BOUNCE_SECTORS	8U

/* Given from the SD DMA completion callbacks below */
static SemaphoreHandle_t xSemaphore;

static BYTE s_bounce[BOUNCE_SECTORS * SD_DEFAULT_BLOCK_SIZE] __attribute__((aligned(32)));

static int is_dma_safe(const void *buff)
{
	const uintptr_t addr = (uintptr_t)buff;
	return (addr & 3U) == 0U && addr >= DMA_SAFE_START && addr < DMA_SAFE_END;
}

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
	if (xSemaphore == NULL)
	{
		xSemaphore = xSemaphoreCreateBinary();
	}
	return stat;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

/* DMA read into a buffer that is safe for DMA */
static DRESULT read_blocks_dma(BYTE *buff, LBA_t sector, UINT count)
{
	DRESULT res = RES_ERROR;
	uint32_t timer;

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

DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber to identify the drive */
	BYTE *buff,		/* Data buffer to store read data */
	LBA_t sector,	/* Start sector in LBA */
	UINT count		/* Number of sectors to read */
)
{
	if (pdrv != DEV_MMC) return RES_PARERR;

	if (is_dma_safe(buff))
	{
		return read_blocks_dma(buff, sector, count);
	}

	while (count > 0U)
	{
		const UINT n = (count < BOUNCE_SECTORS) ? count : BOUNCE_SECTORS;
		const DRESULT res = read_blocks_dma(s_bounce, sector, n);
		if (res != RES_OK)
		{
			return res;
		}
		memcpy(buff, s_bounce, n * SD_DEFAULT_BLOCK_SIZE);
		buff   += n * SD_DEFAULT_BLOCK_SIZE;
		sector += n;
		count  -= n;
	}
	return RES_OK;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if FF_FS_READONLY == 0

DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber to identify the drive */
	const BYTE *buff,	/* Data to be written */
	LBA_t sector,		/* Start sector in LBA */
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
