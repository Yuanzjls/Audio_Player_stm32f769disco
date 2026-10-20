/**
 * @file  audio_out.c
 * @brief SAI1 block A + WM8994 headphone output.
 *
 * Clocking: SAI1 is clocked from PLLI2S (PLLSAI is reserved for the LCD).
 *  - 11.025/22.05/44.1 kHz: PLLI2S VCO 429 MHz / Q 2 / DIVQ 19 = 11.289 MHz
 *  - 8/12/16/24/32/48/96 kHz: PLLI2S VCO 344 MHz / Q 7 / DIVQ 1 = 49.142 MHz
 * The SAI master divider then derives MCLK = 256 x Fs for the codec.
 */
#include "audio_out.h"

#include "main.h"
#include "i2c_bus.h"
#include "log.h"
#include "wm8994/wm8994.h"

/* SAI1 block A pins (MCLK on PG7, SCK/FS/SD on PE5/PE4/PE6) */
#define AUDIO_SAI                 SAI1_Block_A
#define AUDIO_SAI_AF              GPIO_AF6_SAI1
#define AUDIO_DMA_STREAM          DMA2_Stream6
#define AUDIO_DMA_CHANNEL         DMA_CHANNEL_10
#define AUDIO_DMA_IRQn            DMA2_Stream6_IRQn
/* Must be numerically >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY because
 * the callback uses FreeRTOS FromISR APIs. */
#define AUDIO_DMA_IRQ_PRIORITY    1

/* WM8994 AIF1 rate register: AIF1_SR in bits 7:4, AIF1CLK_RATE in bits 3:0 (3 = 256 fs) */
#define WM8994_AIF1_RATE_REG      0x210U
#define AIF1_RATE_12K_256FS       0x0023U
#define AIF1_RATE_24K_256FS       0x0053U

static SAI_HandleTypeDef     s_sai;
static DMA_HandleTypeDef     s_dma;
static audio_out_callback_t  s_callback;
static bool                  s_codec_ready;

static const uint32_t k_supported_rates[] =
{
  8000, 11025, 12000, 16000, 22050, 24000, 32000, 44100, 48000, 96000,
};

bool audio_out_is_rate_supported(uint32_t sample_rate)
{
  for (uint32_t i = 0; i < sizeof(k_supported_rates) / sizeof(k_supported_rates[0]); i++)
  {
    if (k_supported_rates[i] == sample_rate)
    {
      return true;
    }
  }
  return false;
}

static bool is_44k1_family(uint32_t sample_rate)
{
  return (sample_rate % 11025U) == 0U;
}

static bool configure_clock(uint32_t sample_rate)
{
  RCC_PeriphCLKInitTypeDef clk;

  HAL_RCCEx_GetPeriphCLKConfig(&clk);
  clk.PeriphClockSelection = RCC_PERIPHCLK_SAI1;
  clk.Sai1ClockSelection   = RCC_SAI1CLKSOURCE_PLLI2S;
  if (is_44k1_family(sample_rate))
  {
    clk.PLLI2S.PLLI2SN = 429;
    clk.PLLI2S.PLLI2SQ = 2;
    clk.PLLI2SDivQ     = 19;
  }
  else
  {
    clk.PLLI2S.PLLI2SN = 344;
    clk.PLLI2S.PLLI2SQ = 7;
    clk.PLLI2SDivQ     = 1;
  }
  return HAL_RCCEx_PeriphCLKConfig(&clk) == HAL_OK;
}

static bool configure_sai(uint32_t sample_rate)
{
  __HAL_SAI_RESET_HANDLE_STATE(&s_sai);
  s_sai.Instance = AUDIO_SAI;
  __HAL_SAI_DISABLE(&s_sai);

  s_sai.Init.AudioMode      = SAI_MODEMASTER_TX;
  s_sai.Init.Synchro        = SAI_ASYNCHRONOUS;
  s_sai.Init.OutputDrive    = SAI_OUTPUTDRIVE_ENABLE;
  s_sai.Init.NoDivider      = SAI_MASTERDIVIDER_ENABLE;
  s_sai.Init.FIFOThreshold  = SAI_FIFOTHRESHOLD_1QF;
  s_sai.Init.AudioFrequency = sample_rate;
  s_sai.Init.Protocol       = SAI_FREE_PROTOCOL;
  s_sai.Init.DataSize       = SAI_DATASIZE_16;
  s_sai.Init.FirstBit       = SAI_FIRSTBIT_MSB;
  s_sai.Init.ClockStrobing  = SAI_CLOCKSTROBING_FALLINGEDGE;

  s_sai.FrameInit.FrameLength       = 32;
  s_sai.FrameInit.ActiveFrameLength = 16;
  s_sai.FrameInit.FSDefinition      = SAI_FS_CHANNEL_IDENTIFICATION;
  s_sai.FrameInit.FSPolarity        = SAI_FS_ACTIVE_LOW;
  s_sai.FrameInit.FSOffset          = SAI_FS_BEFOREFIRSTBIT;

  s_sai.SlotInit.FirstBitOffset = 0;
  s_sai.SlotInit.SlotSize       = SAI_SLOTSIZE_DATASIZE;
  s_sai.SlotInit.SlotNumber     = 2;
  s_sai.SlotInit.SlotActive     = SAI_SLOTACTIVE_0 | SAI_SLOTACTIVE_1;

  if (HAL_SAI_Init(&s_sai) != HAL_OK)
  {
    return false;
  }
  /* Enable the SAI so that it outputs MCLK: the codec needs it to be configured */
  __HAL_SAI_ENABLE(&s_sai);
  return true;
}

static bool configure_codec(uint32_t sample_rate, uint8_t volume)
{
  bool ok = false;

  i2c_bus_lock();
  if (wm8994_drv.ReadID(AUDIO_I2C_ADDRESS) == WM8994_ID)
  {
    wm8994_drv.Reset(AUDIO_I2C_ADDRESS);
    /* Init() returns the number of register writes that did not read back the
     * written value. Some WM8994 registers are self-clearing (start-up sequencer,
     * DC servo triggers), so it is never 0 even when everything works; ST's BSP
     * ignores it too. The ID check above is what proves the codec is there. */
    (void)wm8994_drv.Init(AUDIO_I2C_ADDRESS, OUTPUT_DEVICE_HEADPHONE, volume, sample_rate);
    /* The ST driver has no 12 / 24 kHz setting (it falls back to 48 kHz) */
    if (sample_rate == 12000U || sample_rate == 24000U)
    {
      AUDIO_IO_Write(AUDIO_I2C_ADDRESS, WM8994_AIF1_RATE_REG,
                     (sample_rate == 12000U) ? AIF1_RATE_12K_256FS : AIF1_RATE_24K_256FS);
    }
    ok = true;
  }
  i2c_bus_unlock();
  return ok;
}

bool audio_out_configure(uint32_t sample_rate, uint8_t volume, audio_out_callback_t callback)
{
  if (!audio_out_is_rate_supported(sample_rate))
  {
    LOG_ERROR("audio: unsupported sample rate %lu Hz", (unsigned long)sample_rate);
    return false;
  }

  audio_out_stop();
  s_callback = callback;

  if (!configure_clock(sample_rate))
  {
    LOG_ERROR("audio: PLLI2S configuration failed");
    return false;
  }
  if (!configure_sai(sample_rate))
  {
    LOG_ERROR("audio: SAI init failed");
    return false;
  }
  if (!configure_codec(sample_rate, volume))
  {
    LOG_ERROR("audio: WM8994 not responding");
    return false;
  }
  s_codec_ready = true;
  return true;
}

bool audio_out_start(int16_t *buffer, uint32_t size_bytes)
{
  /* DMA transfers are counted in 16-bit items and limited to 65535 */
  const uint32_t items = size_bytes / sizeof(int16_t);
  if (!s_codec_ready || items == 0U || items > 0xFFFFU)
  {
    return false;
  }

  i2c_bus_lock();
  wm8994_drv.Play(AUDIO_I2C_ADDRESS, (uint16_t *)buffer, (uint16_t)items);
  i2c_bus_unlock();

  return HAL_SAI_Transmit_DMA(&s_sai, (uint8_t *)buffer, (uint16_t)items) == HAL_OK;
}

bool audio_out_pause(void)
{
  if (!s_codec_ready)
  {
    return false;
  }
  i2c_bus_lock();
  wm8994_drv.Pause(AUDIO_I2C_ADDRESS);
  i2c_bus_unlock();
  return HAL_SAI_DMAPause(&s_sai) == HAL_OK;
}

bool audio_out_resume(void)
{
  if (!s_codec_ready)
  {
    return false;
  }
  i2c_bus_lock();
  wm8994_drv.Resume(AUDIO_I2C_ADDRESS);
  i2c_bus_unlock();
  return HAL_SAI_DMAResume(&s_sai) == HAL_OK;
}

void audio_out_stop(void)
{
  if (!s_codec_ready)
  {
    return;
  }
  HAL_SAI_DMAStop(&s_sai);
  i2c_bus_lock();
  wm8994_drv.Stop(AUDIO_I2C_ADDRESS, CODEC_PDWN_SW);
  i2c_bus_unlock();
}

bool audio_out_set_volume(uint8_t volume)
{
  if (!s_codec_ready)
  {
    return false;
  }
  if (volume > 100U)
  {
    volume = 100U;
  }
  i2c_bus_lock();
  const uint32_t err = wm8994_drv.SetVolume(AUDIO_I2C_ADDRESS, volume);
  i2c_bus_unlock();
  return err == 0U;
}

void audio_out_dma_irq_handler(void)
{
  HAL_DMA_IRQHandler(s_sai.hdmatx);
}

/* ---- HAL callbacks --------------------------------------------------------- */

void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
  if (hsai == &s_sai && s_callback != NULL)
  {
    s_callback(AUDIO_OUT_HALF_DONE);
  }
}

void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai)
{
  if (hsai == &s_sai && s_callback != NULL)
  {
    s_callback(AUDIO_OUT_FULL_DONE);
  }
}

void HAL_SAI_ErrorCallback(SAI_HandleTypeDef *hsai)
{
  (void)hsai;
  /* FIFO underruns are reported here; nothing to do, the DMA keeps running */
}

void HAL_SAI_MspInit(SAI_HandleTypeDef *hsai)
{
  GPIO_InitTypeDef gpio = {0};

  if (hsai->Instance != AUDIO_SAI)
  {
    return;
  }

  __HAL_RCC_SAI1_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  gpio.Mode      = GPIO_MODE_AF_PP;
  gpio.Pull      = GPIO_NOPULL;
  gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = AUDIO_SAI_AF;
  gpio.Pin       = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;   /* FS, SCK, SD */
  HAL_GPIO_Init(GPIOE, &gpio);
  gpio.Pin       = GPIO_PIN_7;                             /* MCLK */
  HAL_GPIO_Init(GPIOG, &gpio);

  s_dma.Instance                 = AUDIO_DMA_STREAM;
  s_dma.Init.Channel             = AUDIO_DMA_CHANNEL;
  s_dma.Init.Direction           = DMA_MEMORY_TO_PERIPH;
  s_dma.Init.PeriphInc           = DMA_PINC_DISABLE;
  s_dma.Init.MemInc              = DMA_MINC_ENABLE;
  s_dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  s_dma.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
  s_dma.Init.Mode                = DMA_CIRCULAR;
  s_dma.Init.Priority            = DMA_PRIORITY_HIGH;
  s_dma.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;
  s_dma.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
  s_dma.Init.MemBurst            = DMA_MBURST_SINGLE;
  s_dma.Init.PeriphBurst         = DMA_PBURST_SINGLE;
  __HAL_LINKDMA(hsai, hdmatx, s_dma);

  HAL_DMA_DeInit(&s_dma);
  if (HAL_DMA_Init(&s_dma) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_NVIC_SetPriority(AUDIO_DMA_IRQn, AUDIO_DMA_IRQ_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(AUDIO_DMA_IRQn);
}
