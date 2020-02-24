/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2016        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

#include "ff.h"			/* Obtains integer types */
#include "diskio.h"		/* Declarations of disk functions */
#include "stm32f769i_discovery_sd.h"
#include "semphr.h"
/* Definitions of physical drive number for each drive */
#define DEV_RAM		1	/* Example: Map Ramdisk to physical drive 0 */
#define DEV_MMC		0	/* Example: Map MMC/SD card to physical drive 1 */
#define DEV_USB		2	/* Example: Map USB MSD to physical drive 2 */

#define SD_TIMEOUT 1000
#define SD_DEFAULT_BLOCK_SIZE 512
/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber to identify the drive */
)
{
	DSTATUS stat;
	int result;

	switch (pdrv) {
	case DEV_RAM :
		//result = RAM_disk_status();

		// translate the reslut code here

		return stat;

	case DEV_MMC :
		 stat = STA_NOINIT;

		  if(BSP_SD_GetCardState() == MSD_OK)
		  {
		    stat &= ~STA_NOINIT;
		  }

		// translate the reslut code here

		return stat;

	case DEV_USB :
		//result = USB_disk_status();

		// translate the reslut code here

		return stat;
	}
	return STA_NOINIT;
}


static SemaphoreHandle_t xSemaphore;
/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
	BYTE pdrv				/* Physical drive nmuber to identify the drive */
)
{
	DSTATUS stat;
	int result;

	switch (pdrv) {
	case DEV_RAM :
		//result = RAM_disk_initialize();

		// translate the reslut code here

		return stat;

	case DEV_MMC :
		stat = BSP_SD_Init();
		xSemaphore = xSemaphoreCreateBinary();
		// translate the reslut code here

		return stat;

	case DEV_USB :
		//result = USB_disk_initialize();

		// translate the reslut code here

		return stat;
	}
	return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/
// To do, may add usb drive in the future, it needs another variable likeulNotificationValue.


DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber to identify the drive */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Start sector in LBA */
	UINT count		/* Number of sectors to read */
)
{
	DRESULT res;
	int result;
	uint32_t timer;

	switch (pdrv) {
	case DEV_RAM :
		// translate the arguments here

		//result = RAM_disk_read(buff, sector, count);

		// translate the reslut code here

		return res;

	case DEV_MMC :
		// translate the arguments here

		res = RES_ERROR;

		if(BSP_SD_ReadBlocks_DMA((uint32_t*)buff,
		(uint32_t) (sector),
		count) == MSD_OK)
		{
			if( xSemaphoreTake( xSemaphore, ( TickType_t ) SD_TIMEOUT ) == pdTRUE )
			{
				/* We were able to obtain the semaphore and can now access the
				shared resource. */
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

		// translate the reslut code here



	case DEV_USB :
		// translate the arguments here

		//result = USB_disk_read(buff, sector, count);

		// translate the reslut code here

		return res;
	}

	return RES_PARERR;
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
	int result;


	switch (pdrv) {
	case DEV_RAM :
		// translate the arguments here

		//result = RAM_disk_write(buff, sector, count);

		// translate the reslut code here

		return res;

	case DEV_MMC :
		// translate the arguments here

		res = RES_ERROR;

		if(BSP_SD_WriteBlocks_DMA((uint32_t*)buff,
		(uint32_t)(sector),
		count) == MSD_OK)
		{
			if( xSemaphoreTake( xSemaphore, ( TickType_t ) SD_TIMEOUT ) == pdTRUE )
			{
				/* We were able to obtain the semaphore and can now access the
				shared resource. */

				while(BSP_SD_GetCardState()!= MSD_OK)
				{
					;
				}
				res = RES_OK;
			}
		}

		  return res;

		// translate the reslut code here



	case DEV_USB :
		// translate the arguments here

		//result = USB_disk_write(buff, sector, count);

		// translate the reslut code here

		return res;
	}

	return RES_PARERR;
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
	int result;

	switch (pdrv) {
	case DEV_RAM :

		// Process of the command for the RAM drive

		return res;

	case DEV_MMC :

		 res = RES_ERROR;
		 HAL_SD_CardInfoTypeDef CardInfo;


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
		    //BSP_SD_GetCardInfo(&CardInfo);
		    *(WORD*)buff = CardInfo.LogBlockSize;
		    res = RES_OK;
		    break;

		  /* Get erase block size in unit of sector (DWORD) */
		  case GET_BLOCK_SIZE :
		    //BSP_SD_GetCardInfo(&CardInfo);
		    *(DWORD*)buff = CardInfo.LogBlockSize / SD_DEFAULT_BLOCK_SIZE;
			res = RES_OK;
		    break;

		  default:
		    res = RES_PARERR;
		  }

		  return res;


	case DEV_USB :

		// Process of the command the USB drive

		return res;
	}

	return RES_PARERR;
}

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

	/* At this point xTaskToNotify should not be NULL as a transmission was
	in progress. */
	//configASSERT( xTaskToNotify != NULL );

	/* Notify the task that the transmission is complete. */
	xSemaphoreGiveFromISR( xSemaphore, &xHigherPriorityTaskWoken );

	/* There are no transmissions in progress, so no tasks to notify. */
	//xTaskToNotify = NULL;

	/* If xHigherPriorityTaskWoken is now set to pdTRUE then a context switch
	should be performed to ensure the interrupt returns directly to the highest
	priority task.  The macro used for this purpose is dependent on the port in
	use and may be called portEND_SWITCHING_ISR(). */
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

/**
  * @brief BSP Rx Transfer completed callbacks
  * @retval None
  */
void BSP_SD_ReadCpltCallback(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	/* At this point xTaskToNotify should not be NULL as a transmission was
	in progress. */
	//configASSERT( xTaskToNotify != NULL );

	/* Notify the task that the transmission is complete. */
	//vTaskNotifyGiveFromISR( xTaskToNotify, &xHigherPriorityTaskWoken );
	xSemaphoreGiveFromISR( xSemaphore, &xHigherPriorityTaskWoken );
	/* There are no transmissions in progress, so no tasks to notify. */
	//xTaskToNotify = NULL;

	/* If xHigherPriorityTaskWoken is now set to pdTRUE then a context switch
	should be performed to ensure the interrupt returns directly to the highest
	priority task.  The macro used for this purpose is dependent on the port in
	use and may be called portEND_SWITCHING_ISR(). */
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}
