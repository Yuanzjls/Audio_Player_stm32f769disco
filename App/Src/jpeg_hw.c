/**
 * @file  jpeg_hw.c
 * @brief Hardware JPEG decoding, fed and drained by DMA (DMA2 stream 3 in,
 *        stream 4 out). The interrupts only swap buffers and wake the decoding
 *        task, which sleeps in between:
 *  - input: two chunks of compressed data, refilled by the task (job->read)
 *    while the codec consumes the other;
 *  - output: two chunks of YCbCr MCUs, handed to the task (job->mcus);
 *  - when the task falls behind, the interrupt pauses that side of the codec
 *    and the task resumes it.
 * The DMA buffers are in DTCM, which is never cached. The hardware decodes
 * baseline JPEG only.
 */
#include "jpeg_hw.h"

#include <string.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "log.h"
#include "main.h"

#define IN_CHUNK          4096U
#define OUT_CHUNK         (768U * 16U) /* whole MCUs of every layout (64/192/256/384 bytes) */
#define DECODE_TIMEOUT_MS 3000U
#define IRQ_PRIORITY      6
#define MAX_SIDE          8192U

/* A DMA buffer and how much of it is valid (0 = empty / free) */
typedef struct
{
  uint8_t *data;
  volatile uint32_t len;
} chunk_t;

static JPEG_HandleTypeDef s_jpeg;
static DMA_HandleTypeDef  s_dma_in;
static DMA_HandleTypeDef  s_dma_out;
static SemaphoreHandle_t  s_lock;

/* DMA buffers: DTCM, uncached */
static uint8_t s_in_mem[2][IN_CHUNK] DMA_BUFFER;
static uint8_t s_out_mem[2][OUT_CHUNK] DMA_BUFFER;
static chunk_t s_in[2]  = { { s_in_mem[0], 0 }, { s_in_mem[1], 0 } };
static chunk_t s_out[2] = { { s_out_mem[0], 0 }, { s_out_mem[1], 0 } };

/* Shared between the interrupts and the task (changed with interrupts masked) */
static TaskHandle_t     s_waiter;        /* task running the decode       */
static volatile uint8_t s_in_read;       /* input chunk the codec reads   */
static volatile uint8_t s_out_write;     /* output chunk the codec writes */
static volatile bool    s_in_paused;
static volatile bool    s_out_paused;
static volatile bool    s_info_ready;
static volatile bool    s_finished;      /* decoded, or failed            */
static volatile bool    s_failed;
static JPEG_ConfTypeDef s_info;

/* Task side of the current decode */
static const jpeg_hw_job_t *s_job;
static bool                 s_in_done;   /* job->read() has no more data */
static uint8_t              s_in_fill;   /* input chunk the task fills next  */
static uint8_t              s_out_read;  /* output chunk the task takes next */
static jpeg_hw_layout_t     s_layout;
static bool                 s_layout_ok;

/* ---- set-up -------------------------------------------------------------------- */

static void dma_init(DMA_HandleTypeDef *dma, DMA_Stream_TypeDef *stream, uint32_t direction)
{
  dma->Instance                 = stream;
  dma->Init.Channel             = DMA_CHANNEL_9;
  dma->Init.Direction           = direction;
  dma->Init.PeriphInc           = DMA_PINC_DISABLE;
  dma->Init.MemInc              = DMA_MINC_ENABLE;
  dma->Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  dma->Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
  dma->Init.Mode                = DMA_NORMAL;
  dma->Init.Priority            = DMA_PRIORITY_HIGH;
  dma->Init.FIFOMode            = DMA_FIFOMODE_ENABLE;
  dma->Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
  dma->Init.MemBurst            = DMA_MBURST_INC4;
  dma->Init.PeriphBurst         = DMA_PBURST_INC4;
  HAL_DMA_DeInit(dma);
  HAL_DMA_Init(dma);
}

void HAL_JPEG_MspInit(JPEG_HandleTypeDef *hjpeg)
{
  __HAL_RCC_JPEG_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  dma_init(&s_dma_in, DMA2_Stream3, DMA_MEMORY_TO_PERIPH);
  __HAL_LINKDMA(hjpeg, hdmain, s_dma_in);
  dma_init(&s_dma_out, DMA2_Stream4, DMA_PERIPH_TO_MEMORY);
  __HAL_LINKDMA(hjpeg, hdmaout, s_dma_out);

  HAL_NVIC_SetPriority(JPEG_IRQn, IRQ_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(JPEG_IRQn);
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, IRQ_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, IRQ_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);
}

void JPEG_IRQHandler(void)          { HAL_JPEG_IRQHandler(&s_jpeg); }
void DMA2_Stream3_IRQHandler(void)  { HAL_DMA_IRQHandler(s_jpeg.hdmain); }
void DMA2_Stream4_IRQHandler(void)  { HAL_DMA_IRQHandler(s_jpeg.hdmaout); }

void jpeg_hw_init(void)
{
  s_lock = xSemaphoreCreateMutex();
  configASSERT(s_lock != NULL);
  s_jpeg.Instance = JPEG;
  if (HAL_JPEG_Init(&s_jpeg) != HAL_OK)
  {
    LOG_ERROR("jpeg: codec initialisation failed");
  }
}

/* ---- codec callbacks (interrupt context: swap buffers, wake the task) -------- */

static void wake_task_from_isr(void)
{
  BaseType_t woken = pdFALSE;
  vTaskNotifyGiveFromISR(s_waiter, &woken);
  portYIELD_FROM_ISR(woken);
}

void HAL_JPEG_InfoReadyCallback(JPEG_HandleTypeDef *hjpeg, JPEG_ConfTypeDef *info)
{
  (void)hjpeg;
  s_info       = *info;
  s_info_ready = true;
  wake_task_from_isr();
}

/* The codec has consumed the input chunk: go on with the other one */
void HAL_JPEG_GetDataCallback(JPEG_HandleTypeDef *hjpeg, uint32_t consumed)
{
  (void)consumed;
  s_in[s_in_read].len = 0;
  s_in_read ^= 1U;
  if (s_in[s_in_read].len != 0U)
  {
    HAL_JPEG_ConfigInputBuffer(hjpeg, s_in[s_in_read].data, s_in[s_in_read].len);
  }
  else
  {
    HAL_JPEG_Pause(hjpeg, JPEG_PAUSE_RESUME_INPUT);   /* resumed by the task */
    s_in_paused = true;
  }
  wake_task_from_isr();
}

/* An output chunk is full (or the last, partial one): continue in the other */
void HAL_JPEG_DataReadyCallback(JPEG_HandleTypeDef *hjpeg, uint8_t *data, uint32_t len)
{
  (void)data;
  s_out[s_out_write].len = len;
  s_out_write ^= 1U;
  if (s_out[s_out_write].len != 0U)
  {
    HAL_JPEG_Pause(hjpeg, JPEG_PAUSE_RESUME_OUTPUT);  /* resumed by the task */
    s_out_paused = true;
  }
  HAL_JPEG_ConfigOutputBuffer(hjpeg, s_out[s_out_write].data, OUT_CHUNK);
  wake_task_from_isr();
}

void HAL_JPEG_DecodeCpltCallback(JPEG_HandleTypeDef *hjpeg)
{
  (void)hjpeg;
  s_finished = true;
  wake_task_from_isr();
}

void HAL_JPEG_ErrorCallback(JPEG_HandleTypeDef *hjpeg)
{
  (void)hjpeg;
  s_failed   = true;
  s_finished = true;
  wake_task_from_isr();
}

/* ---- task side ----------------------------------------------------------------- */

/* Read the next piece of the compressed data into an empty input chunk */
static void fill_input(void)
{
  chunk_t *c = &s_in[s_in_fill];
  if (c->len != 0U || s_in_done)
  {
    return;
  }
  const uint32_t got = s_job->read(s_job->ctx, c->data, IN_CHUNK);
  if (got == 0U)
  {
    s_in_done = true;
    return;
  }
  /* The codec takes whole words: pad the end of the data */
  const uint32_t padded = (got + 3U) & ~3U;
  memset(&c->data[got], 0, padded - got);
  c->len = padded;
  s_in_fill ^= 1U;
}

/* MCU layout from the header; false if the picture cannot be decoded */
static bool make_layout(const JPEG_ConfTypeDef *info, jpeg_hw_layout_t *l)
{
  memset(l, 0, sizeof(*l));
  l->width  = info->ImageWidth;
  l->height = info->ImageHeight;

  if (info->ColorSpace == JPEG_GRAYSCALE_COLORSPACE)
  {
    l->gray = true; l->mcu_w = 8; l->mcu_h = 8; l->y_blocks = 1; l->mcu_bytes = 64;
  }
  else if (info->ColorSpace == JPEG_YCBCR_COLORSPACE)
  {
    switch (info->ChromaSubsampling)
    {
      case JPEG_420_SUBSAMPLING: l->mcu_w = 16; l->mcu_h = 16; l->y_blocks = 4; break;
      case JPEG_422_SUBSAMPLING: l->mcu_w = 16; l->mcu_h = 8;  l->y_blocks = 2; break;
      default:                   l->mcu_w = 8;  l->mcu_h = 8;  l->y_blocks = 1; break;
    }
    l->mcu_bytes = (l->y_blocks + 2U) * 64U;
  }
  else
  {
    LOG_WARN("jpeg: CMYK pictures are not supported");
    return false;
  }
  if (l->width == 0U || l->height == 0U || l->width > MAX_SIDE || l->height > MAX_SIDE)
  {
    LOG_WARN("jpeg: %lux%lu picture not supported", (unsigned long)l->width, (unsigned long)l->height);
    return false;
  }
  l->mcus_per_row = (l->width + l->mcu_w - 1U) / l->mcu_w;
  return true;
}

/* Hand the full output chunks, in order, to the job and give them back to the codec */
static void drain_output(void)
{
  while (s_out[s_out_read].len != 0U)
  {
    chunk_t *c = &s_out[s_out_read];
    if (s_layout_ok)
    {
      const uint32_t whole = c->len - c->len % s_layout.mcu_bytes;
      if (whole != 0U)
      {
        s_job->mcus(s_job->ctx, c->data, whole);
      }
    }
    c->len = 0;
    s_out_read ^= 1U;
  }
}

/* Resume a side of the codec that the interrupt paused, once it can go on */
static void resume_codec(void)
{
  taskENTER_CRITICAL();
  if (s_out_paused && s_out[s_out_write].len == 0U)
  {
    s_out_paused = false;
    HAL_JPEG_Resume(&s_jpeg, JPEG_PAUSE_RESUME_OUTPUT);
  }
  if (s_in_paused && s_in[s_in_read].len != 0U)
  {
    s_in_paused = false;
    HAL_JPEG_ConfigInputBuffer(&s_jpeg, s_in[s_in_read].data, s_in[s_in_read].len);
    HAL_JPEG_Resume(&s_jpeg, JPEG_PAUSE_RESUME_INPUT);
  }
  taskEXIT_CRITICAL();
}

/* Stop the codec and its DMA, and start again from a clean state */
static void reset_codec(void)
{
  HAL_JPEG_Abort(&s_jpeg);
  HAL_JPEG_DeInit(&s_jpeg);
  if (HAL_JPEG_Init(&s_jpeg) != HAL_OK)
  {
    LOG_ERROR("jpeg: codec re-initialisation failed");
  }
}

jpeg_hw_result_t jpeg_hw_decode(const jpeg_hw_job_t *job)
{
  xSemaphoreTake(s_lock, portMAX_DELAY);

  s_job       = job;
  s_waiter    = xTaskGetCurrentTaskHandle();
  s_layout_ok = false;
  s_in_done   = false;
  s_in[0].len = s_in[1].len = 0;
  s_out[0].len = s_out[1].len = 0;
  s_in_read = s_in_fill = 0;
  s_out_write = s_out_read = 0;
  s_in_paused = s_out_paused = false;
  s_info_ready = s_finished = s_failed = false;
  (void)ulTaskNotifyTake(pdTRUE, 0);

  jpeg_hw_result_t result = JPEG_HW_ERROR;
  fill_input();
  fill_input();
  if (s_in[0].len != 0U &&
      HAL_JPEG_Decode_DMA(&s_jpeg, s_in[0].data, s_in[0].len, s_out[0].data, OUT_CHUNK) == HAL_OK)
  {
    bool timed_out   = false;
    bool rejected    = false;
    bool layout_done = false;
    while (!s_finished)
    {
      if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(DECODE_TIMEOUT_MS)) == 0U)
      {
        timed_out = true;
        break;
      }
      /* Once per picture: an unwanted or unsupported one stops the decoding at once */
      if (s_info_ready && !layout_done)
      {
        layout_done = true;
        s_layout_ok = make_layout(&s_info, &s_layout) && job->begin(job->ctx, &s_layout);
        if (!s_layout_ok)
        {
          rejected = true;
          break;
        }
      }
      drain_output();
      fill_input();
      resume_codec();
    }
    drain_output();   /* the last chunk */

    if (rejected)
    {
      result = JPEG_HW_REJECTED;
    }
    else if (timed_out || s_failed || !s_layout_ok)
    {
      LOG_WARN("jpeg: decoding %s (error 0x%lx)", timed_out ? "timed out" : "failed",
               (unsigned long)HAL_JPEG_GetError(&s_jpeg));
    }
    else
    {
      result = JPEG_HW_OK;
    }
    if (result != JPEG_HW_OK)
    {
      reset_codec();
    }
  }

  s_job = NULL;
  xSemaphoreGive(s_lock);
  return result;
}
