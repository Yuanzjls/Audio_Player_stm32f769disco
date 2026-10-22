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
#include "main.h"
#include "log.h"
#include "stm32f769i_discovery_sd.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

/* Physical drive number of the SD card */
#define DEV_MMC		0

#define SD_TIMEOUT 1000
#define SD_ATTEMPTS 3U		/* a failed transfer is retried this many times in total */
#define SD_DEFAULT_BLOCK_SIZE 512

/* DTCM is never cached, so the SD DMA can write there directly. Anything else
 * (cacheable SRAM1/2 and SDRAM) goes through this bounce buffer in DTCM. */
#define DMA_SAFE_START	0x20000000UL
#define DMA_SAFE_END	0x20020000UL
#define BOUNCE_SECTORS	32U		/* 16 KB: few commands per transfer */

/* Given from the SD completion and error callbacks below */
static SemaphoreHandle_t xSemaphore;
static volatile uint32_t s_sd_error;	/* HAL error code of the last transfer, 0 = none */

extern SD_HandleTypeDef uSdHandle;		/* BSP */

static BYTE s_bounce[BOUNCE_SECTORS * SD_DEFAULT_BLOCK_SIZE] DMA_BUFFER;

static int is_dma_safe(const void *buff, UINT count)
{
	const uintptr_t addr = (uintptr_t)buff;
	return (addr & 3U) == 0U && addr >= DMA_SAFE_START &&
	       addr + (uintptr_t)count * SD_DEFAULT_BLOCK_SIZE <= DMA_SAFE_END;
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

	/* No card in the slot: FatFs mounts the volume again on the next access
	 * once there is one */
	if (BSP_SD_IsDetected() != SD_PRESENT) return STA_NOINIT | STA_NODISK;

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

/* After a transfer the card is usually ready at once; otherwise sleep 1 ms
 * between checks instead of spinning. Returns 0 when ready, -1 on timeout. */
static int wait_card_ready(void)
{
	const TickType_t start = xTaskGetTickCount();

	while (BSP_SD_GetCardState() != MSD_OK)
	{
		if ((xTaskGetTickCount() - start) >= pdMS_TO_TICKS(SD_TIMEOUT))
		{
			return -1;
		}
		vTaskDelay(1);
	}
	return 0;
}

/* One DMA read into a buffer that is safe for DMA */
static DRESULT read_blocks_once(BYTE *buff, LBA_t sector, UINT count)
{
	/* Drop a completion left over from an earlier, abandoned transfer */
	xSemaphoreTake(xSemaphore, 0);
	s_sd_error = 0;

	if (wait_card_ready() != 0)
	{
		return RES_NOTRDY;
	}
	if (BSP_SD_ReadBlocks_DMA((uint32_t *)buff, (uint32_t)sector, count) != MSD_OK)
	{
		s_sd_error = uSdHandle.ErrorCode;
		return RES_ERROR;
	}

	/* Woken by BSP_SD_ReadCpltCallback() or HAL_SD_ErrorCallback() */
	if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(SD_TIMEOUT)) != pdTRUE)
	{
		s_sd_error = SDMMC_ERROR_TIMEOUT;
		return RES_ERROR;
	}
	return (s_sd_error == 0U) ? RES_OK : RES_ERROR;
}

/* DMA read, retried after an error (a CRC error, a FIFO overrun, or a command
 * the card did not answer). The first retry follows an abort; the last one a
 * complete re-initialisation of the card. */
static DRESULT read_blocks_dma(BYTE *buff, LBA_t sector, UINT count)
{
	DRESULT res = RES_ERROR;

	for (UINT attempt = 1; attempt <= SD_ATTEMPTS; attempt++)
	{
		const TickType_t start = xTaskGetTickCount();
		res = read_blocks_once(buff, sector, count);
		if (res == RES_OK)
		{
			if (attempt > 1U)
			{
				LOG_INFO("sd: read of sector %lu ok on attempt %u", (unsigned long)sector, attempt);
			}
			break;
		}
		LOG_WARN("sd: read of %u sector(s) at %lu failed after %lu ms (error 0x%lx)", count,
		         (unsigned long)sector, (unsigned long)(xTaskGetTickCount() - start) * portTICK_PERIOD_MS,
		         (unsigned long)s_sd_error);
		if (BSP_SD_IsDetected() != SD_PRESENT)
		{
			break;    /* taken out: no point retrying */
		}

		if (attempt + 1U < SD_ATTEMPTS)
		{
			HAL_SD_Abort(&uSdHandle);
		}
		else if (attempt + 1U == SD_ATTEMPTS)
		{
			BSP_SD_DeInit();
			if (BSP_SD_Init() != MSD_OK)
			{
				LOG_ERROR("sd: card re-initialisation failed");
				break;
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
	/* Card taken out: fail at once, not after the retries */
	if (BSP_SD_IsDetected() != SD_PRESENT) return RES_NOTRDY;

	if (is_dma_safe(buff, count))
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

/* One DMA write from a buffer that is safe for DMA; waits until the card has
 * programmed the data */
static DRESULT write_blocks_once(const BYTE *buff, LBA_t sector, UINT count)
{
	xSemaphoreTake(xSemaphore, 0);
	s_sd_error = 0;

	if (wait_card_ready() != 0)
	{
		return RES_NOTRDY;
	}
	if (BSP_SD_WriteBlocks_DMA((uint32_t *)buff, (uint32_t)sector, count) != MSD_OK)
	{
		s_sd_error = uSdHandle.ErrorCode;
		return RES_ERROR;
	}
	/* Woken by BSP_SD_WriteCpltCallback() or HAL_SD_ErrorCallback() */
	if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(SD_TIMEOUT)) != pdTRUE)
	{
		s_sd_error = SDMMC_ERROR_TIMEOUT;
		return RES_ERROR;
	}
	if (s_sd_error != 0U)
	{
		return RES_ERROR;
	}
	return (wait_card_ready() == 0) ? RES_OK : RES_ERROR;
}

/* DMA write, retried after an abort if it fails */
static DRESULT write_blocks_dma(const BYTE *buff, LBA_t sector, UINT count)
{
	DRESULT res = RES_ERROR;

	for (UINT attempt = 1; attempt <= SD_ATTEMPTS; attempt++)
	{
		res = write_blocks_once(buff, sector, count);
		if (res == RES_OK)
		{
			break;
		}
		LOG_WARN("sd: write of %u sector(s) at %lu failed (error 0x%lx)", count, (unsigned long)sector,
		         (unsigned long)s_sd_error);
		HAL_SD_Abort(&uSdHandle);
	}
	return res;
}

DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber to identify the drive */
	const BYTE *buff,	/* Data to be written */
	LBA_t sector,		/* Start sector in LBA */
	UINT count			/* Number of sectors to write */
)
{
	if (pdrv != DEV_MMC) return RES_PARERR;
	if (BSP_SD_IsDetected() != SD_PRESENT) return RES_NOTRDY;

	if (is_dma_safe(buff, count))
	{
		return write_blocks_dma(buff, sector, count);
	}

	/* Cached memory: the DMA would read what is not yet written back from the
	 * cache, so the data goes through the bounce buffer in uncached DTCM */
	while (count > 0U)
	{
		const UINT n = (count < BOUNCE_SECTORS) ? count : BOUNCE_SECTORS;
		memcpy(s_bounce, buff, n * SD_DEFAULT_BLOCK_SIZE);
		const DRESULT res = write_blocks_dma(s_bounce, sector, n);
		if (res != RES_OK)
		{
			return res;
		}
		buff   += n * SD_DEFAULT_BLOCK_SIZE;
		sector += n;
		count  -= n;
	}
	return RES_OK;
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
  * @brief SD transfer error (CRC, timeout, FIFO overrun, DMA error): wake the
  *        waiting task at once instead of letting it time out
  * @retval None
  */
void HAL_SD_ErrorCallback(SD_HandleTypeDef *hsd)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	s_sd_error = (hsd->ErrorCode != 0U) ? hsd->ErrorCode : SDMMC_ERROR_GENERAL_UNKNOWN_ERR;
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
