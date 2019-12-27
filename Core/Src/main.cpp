/* USER CODE BEGIN Header */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

#include <main.h>
#include <string.h>
#include <ff.h>
#include <decode_polling.h>
#include "jpeg_utils.h"
#include "GifDecoder.h"

//#include "stm32f769i_discovery_lcd.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

extern LTDC_HandleTypeDef hltdc_discovery;
static DMA2D_HandleTypeDef   hdma2d;
extern DSI_HandleTypeDef hdsi_discovery;
DMA2D_HandleTypeDef Dma2dHandle;
DSI_VidCfgTypeDef hdsivideo_handle;
DSI_CmdCfgTypeDef CmdCfg;
DSI_LPCmdTypeDef LPCmd;
DSI_PLLInitTypeDef dsiPllInit;

#define JPEG_SOI_MARKER (0xFFD8) /* JPEG Start Of Image marker*/
#define JPEG_SOI_MARKER_BYTE0 (JPEG_SOI_MARKER & 0xFF)
#define JPEG_SOI_MARKER_BYTE1 ((JPEG_SOI_MARKER >> 8) & 0xFF)
/*JPEG Related variable
 */
static JPEG_HandleTypeDef     JPEG_Handle;
static JPEG_ConfTypeDef       JPEG_Info;
extern __IO uint32_t Previous_FrameSize;

static RCC_PeriphCLKInitTypeDef  PeriphClkInitStruct;
#define VSYNC           1
#define VBP             1
#define VFP             1
#define VACT            480
#define HSYNC           1
#define HBP             1
#define HFP             1
#define HACT            400      /* !!!! SCREEN DIVIDED INTO 2 AREAS !!!! */

#define LAYER0_ADDRESS  (LCD_FB_START_ADDRESS)
#define LAYER1_ADDRESS  (LCD_FB_START_ADDRESS + 800 * 480 * 4)
#define IMAGEBUFF_ADDRESS (LAYER1_ADDRESS + 800 * 480 * 4)
#define GIFIMAGE_ADDRESS (IMAGEBUFF_ADDRESS + 800 * 480 * 4)
#define GIFIMAGEBU_ADDRESS (GIFIMAGE_ADDRESS + 800 * 480)

#define INVALID_AREA      0
#define LEFT_AREA         1
#define RIGHT_AREA        2
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/



RTC_HandleTypeDef hrtc;

FIL JPEG_File;  /* File object */
/* USER CODE BEGIN PV */
/*GIF Decode Varible part*/

/* template parameters are maxGifWidth, maxGifHeight, lzwMaxBits
 *
 * The lzwMaxBits value of 12 supports all GIFs, but uses 16kB RAM
 * lzwMaxBits can be set to 10 or 11 for smaller displays to save RAM, but use 12 for large displays
 * All 32x32-pixel GIFs tested so far work with 11, most work with 10
 */
const uint16_t kMatrixWidth = 500;        // known working: 32, 64, 96, 128
const uint16_t kMatrixHeight = 450;       // known working: 16, 32, 48, 64
GifDecoder<kMatrixWidth, kMatrixHeight, 12> decoder;//((uint8_t *)GIFIMAGE_ADDRESS, (uint8_t *)GIFIMAGEBU_ADDRESS);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
 void SystemClock_Config(void);
static void MX_RTC_Init(void);
static void CopyPicture(uint32_t *pSrc,
                           uint32_t *pDst,
                           uint16_t x,
                           uint16_t y,
                           uint16_t xsize,
                           uint16_t ysize);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void DMA2D_Config(uint32_t AlphaInvertConfig)
{
  HAL_StatusTypeDef hal_status = HAL_OK;

  /* Configure the DMA2D Mode, Color Mode and output offset */
  Dma2dHandle.Init.Mode         = DMA2D_M2M_BLEND; /* DMA2D mode Memory to Memory with Blending */
  Dma2dHandle.Init.ColorMode    = DMA2D_OUTPUT_RGB565; /* output format of DMA2D */
  Dma2dHandle.Init.OutputOffset = 0x0;  /* No offset in output */
  Dma2dHandle.Init.AlphaInverted = DMA2D_REGULAR_ALPHA;  /* No Output Alpha Inversion*/  
  Dma2dHandle.Init.RedBlueSwap   = DMA2D_RB_REGULAR;     /* No Output Red & Blue swap */       

  /* DMA2D Callbacks Configuration */
  Dma2dHandle.XferCpltCallback  = NULL;
  Dma2dHandle.XferErrorCallback = NULL;

  /* Foreground layer Configuration */
  Dma2dHandle.LayerCfg[1].AlphaMode = DMA2D_REPLACE_ALPHA;
  Dma2dHandle.LayerCfg[1].InputAlpha = 0x40;
  Dma2dHandle.LayerCfg[1].InputColorMode = DMA2D_INPUT_RGB565;
  Dma2dHandle.LayerCfg[1].InputOffset = 0x0; /* No offset in input */
  Dma2dHandle.LayerCfg[1].RedBlueSwap = DMA2D_RB_REGULAR; /* No ForeGround Red a Blue swap */
  Dma2dHandle.LayerCfg[1].AlphaInverted = AlphaInvertConfig; /* ForeGround Alpha inversion setting */
  
  /* Background layer Configuration */
  Dma2dHandle.LayerCfg[0].AlphaMode = DMA2D_REPLACE_ALPHA;
  Dma2dHandle.LayerCfg[0].InputAlpha = 0x40; /* fully opaque */
  Dma2dHandle.LayerCfg[0].InputColorMode = DMA2D_INPUT_RGB565;
  Dma2dHandle.LayerCfg[0].InputOffset = 0x0; /* No offset in input */
  Dma2dHandle.LayerCfg[0].RedBlueSwap = DMA2D_RB_REGULAR; /* No BackGround Red a Blue swap */
  Dma2dHandle.LayerCfg[0].AlphaInverted = !AlphaInvertConfig; /* No BackGround Alpha inversion */  

  Dma2dHandle.Instance = DMA2D;

  /* DMA2D Initialization */
  hal_status = HAL_DMA2D_DeInit(&Dma2dHandle);
//  OnError_Handler(hal_status != HAL_OK);
  
  hal_status = HAL_DMA2D_Init(&Dma2dHandle);
//  OnError_Handler(hal_status != HAL_OK);

  /* Apply DMA2D Foreground configuration */
  HAL_DMA2D_ConfigLayer(&Dma2dHandle, 1);

  /* Apply DMA2D Background configuration */
  HAL_DMA2D_ConfigLayer(&Dma2dHandle, 0);
}
#if 0
#define PATTERN_SEARCH_BUFFERSIZE 512
uint8_t PatternSearchBuffer[PATTERN_SEARCH_BUFFERSIZE];
uint32_t JPEG_FindFrameOffset(uint32_t offset, FIL *file)
{
  uint32_t index = offset, i, readSize=0;

  do
  {
    if (f_size(file) <= (index+1))
    {
      return 0;
    }
    f_lseek(file, index);
    f_read(file, PatternSearchBuffer, PATTERN_SEARCH_BUFFERSIZE, (UINT *)&readSize);
    if (readSize!=0)
    {
      for (i=0; i<(readSize-1); i++)
      {
        if ((PatternSearchBuffer[i]==JPEG_SOI_MARKER_BYTE1) && (PatternSearchBuffer[i+1] == JPEG_SOI_MARKER_BYTE0))
        {
          return index + i;
        }
      }
      index += (readSize - 1);
    }
  } while (readSize!=0);

  return 0;
}

#endif

FIL fi;

/*gif call back functions start */
void screenClearCallback(void)
{
	BSP_LCD_Clear(LCD_COLOR_BLACK);
}
void updateScreenCallback(void)
{
	uint32_t xPos = (BSP_LCD_GetXSize() - decoder.GetGifWidth())/2;
	uint32_t yPos = (BSP_LCD_GetYSize() - decoder.GetGifHeight())/2;
	CopyPicture((uint32_t *)IMAGEBUFF_ADDRESS, (uint32_t *)LAYER1_ADDRESS, xPos, yPos, decoder.GetGifWidth(), decoder.GetGifHeight());
}
void drawPixelCallback(int16_t x, int16_t y, uint8_t red, uint8_t green, uint8_t blue)
{
	*((uint32_t*)(IMAGEBUFF_ADDRESS+(x+y* (decoder.GetGifWidth()))*4)) = ((uint16_t)red << 16) | ((uint16_t)green << 8) | (blue);
}

bool fileSeekCallback(unsigned long position) 
{
	if (f_lseek(&fi, position) == FR_OK)
	{
		return true;
	}
	else
	{
		return false;
	}
}

unsigned long filePositionCallback(void) 
{
    return f_tell(&fi);
}
int fileReadCallback(void) 
{
    uint8_t b;
    FRESULT fr;
    UINT ReadLen;
    fr = f_read(&fi, &b, 1, &ReadLen);
    if (fr == FR_OK && ReadLen == 1)
    {
      return b;
    }
    else
    {
      return -1;
    }
}
#define BlockReadSize 512
#define BlockBuffer PatternSearchBuffer
int fileReadBlockCallback(void * buffer, int numberOfBytes) 
{
    uint8_t b;
    FRESULT fr;
    UINT ReadLen;

    fr = f_read(&fi, buffer, numberOfBytes, &ReadLen);
    if (fr == FR_OK)
    {
      return ReadLen;
    }
    else
    {
      return -1;
    }
    
}
/*gif call back functions end */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
uint32_t FrameOffset=0;
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */
	FATFS fs;

	uint8_t SD_state = MSD_OK, res;
	DIR dir;
	uint32_t length=0;
	uint32_t AlphaInvertConfig;
	char temp[255];
  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */

	  MX_RTC_Init();
	  BSP_LCD_Init();
    
	  BSP_SDRAM_Init();


	  /* Initialize LTDC layer 0 iused for Hint */
	  BSP_LCD_LayerDefaultInit(1, LAYER1_ADDRESS);
	  BSP_LCD_SelectLayer(1);
	  BSP_LCD_Clear(LCD_COLOR_BLACK);

	  BSP_LED_Init(LED1);
	  /*##-1- JPEG Initialization ################################################*/
	  /* Init The JPEG Color Look Up Tables used for YCbCr to RGB conversion   */
	  
	  /* Init the HAL JPEG driver */
	   JPEG_Handle.Instance = JPEG;
	   

  /* USER CODE BEGIN 2 */
	   // gif decode callback function set
	decoder.setScreenClearCallback(screenClearCallback);
	decoder.setUpdateScreenCallback(updateScreenCallback);
	decoder.setDrawPixelCallback(drawPixelCallback);

	decoder.setFileSeekCallback(fileSeekCallback);
	decoder.setFilePositionCallback(filePositionCallback);
	decoder.setFileReadCallback(fileReadCallback);
	decoder.setFileReadBlockCallback(fileReadBlockCallback);
  /* USER CODE END 2 */
    //BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
    SD_state = BSP_SD_Init();

    FRESULT fr;
    DIR dj;
    FILINFO fno;

    if (f_mount(&fs,(char*)"",1) == FR_OK)
    {
     
      if (f_open(&fi, "gif.gif", FA_READ) == FR_OK)
      {
    	decoder.startDecoding();
        while(1)
        {
          decoder.decodeFrame();
        }
      }
      
    }

    while(1)
    {
      BSP_LED_Toggle(LED1);
      HAL_Delay(500);
	}
    /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
 void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /**Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /**Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC|RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 384;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 7;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV2;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}




/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */
  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */
  /* USER CODE END RTC_Init 1 */
  /**Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
  /* USER CODE END RTC_Init 2 */

}


/* USER CODE BEGIN 4 */
/**
  * @brief  End of Refresh DSI callback.
  * @param  hdsi: pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @retval None
  */
void HAL_DSI_EndOfRefreshCallback(DSI_HandleTypeDef *hdsi)
{

}


/**
  * @brief  Converts a line to an ARGB8888 pixel format.
  * @param  pSrc: Pointer to source buffer
  * @param  pDst: Output color
  * @param  xSize: Buffer width
  * @param  ColorMode: Input color mode
  * @retval None
  */
static void CopyPicture(uint32_t *pSrc, uint32_t *pDst, uint16_t x, uint16_t y, uint16_t xsize, uint16_t ysize)
{

  uint32_t destination = (uint32_t)pDst + (y * 800 + x) * 4;
  uint32_t source      = (uint32_t)pSrc;

  /*##-1- Configure the DMA2D Mode, Color Mode and output offset #############*/
  hdma2d.Init.Mode         = DMA2D_M2M_PFC;
  hdma2d.Init.ColorMode    = DMA2D_OUTPUT_ARGB8888;
  hdma2d.Init.OutputOffset = 800 - xsize;
  hdma2d.Init.AlphaInverted = DMA2D_REGULAR_ALPHA;  /* No Output Alpha Inversion*/
  hdma2d.Init.RedBlueSwap   = DMA2D_RB_REGULAR;     /* No Output Red & Blue swap */

  /*##-2- DMA2D Callbacks Configuration ######################################*/
  hdma2d.XferCpltCallback  = NULL;

  /*##-3- Foreground Configuration ###########################################*/
  hdma2d.LayerCfg[1].AlphaMode = DMA2D_REPLACE_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha = 0xff;
  hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_ARGB8888;
  hdma2d.LayerCfg[1].InputOffset = 0;
  hdma2d.LayerCfg[1].RedBlueSwap = DMA2D_RB_REGULAR; /* No ForeGround Red/Blue swap */
  hdma2d.LayerCfg[1].AlphaInverted = DMA2D_REGULAR_ALPHA; /* No ForeGround Alpha inversion */

  hdma2d.Instance          = DMA2D;

  /* DMA2D Initialization */
  if(HAL_DMA2D_Init(&hdma2d) == HAL_OK)
  {
    if(HAL_DMA2D_ConfigLayer(&hdma2d, 1) == HAL_OK)
    {
      if (HAL_DMA2D_Start(&hdma2d, source, destination, xsize, ysize) == HAL_OK)
      {
        /* Polling For DMA transfer */
        HAL_DMA2D_PollForTransfer(&hdma2d, 100);
      }
    }
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
