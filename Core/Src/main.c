/**
 * @file  main.c
 * @brief System start-up: MPU/caches, clocks, SDRAM, logging, then two tasks:
 *        - "player": SD card + WAV streaming to the audio codec (App/player.c)
 *        - "gui":    LVGL display, touch and the player screen (App/ui.c)
 */
#include "main.h"

#ifndef FIRMWARE_VERSION
#define FIRMWARE_VERSION "unknown"   /* normally set by CMakeLists.txt (git describe) */
#endif

#include <stdbool.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "lvgl.h"

#include "display.h"
#include "i2c_bus.h"
#include "jpeg_hw.h"
#include "video_player.h"
#include "log.h"
#include "lv_fs_sd.h"
#include "player.h"
#include "sysmon.h"
#include "stm32f769i_discovery_sdram.h"
#include "touch.h"
#include "shell.h"
#ifdef WITH_DOOM
#include "doom_port.h"
#endif

#define INITIAL_VOLUME      20      /* 0..100 */

#define GUI_TASK_STACK      4096    /* words: TrueType rendering needs some stack */
#define GUI_TASK_PRIORITY   1
#define GUI_MAX_SLEEP_MS    100     /* upper bound; LVGL usually asks to run sooner */

static void mpu_config(void);
static void cpu_cache_enable(void);
static void system_clock_config(void);
static void gui_task(void *arg);

const char *firmware_version(void)
{
  return FIRMWARE_VERSION;
}

int main(void)
{
  mpu_config();
  cpu_cache_enable();

  HAL_Init();
  system_clock_config();
  sysmon_init();

  BSP_LED_Init(LED_RED);
  BSP_LED_Init(LED_GREEN);

  log_init();
  LOG_INFO("Audio player %s starting (%lu MHz)", FIRMWARE_VERSION, (unsigned long)(SystemCoreClock / 1000000U));

  /* Frame buffers and the LVGL heap live in SDRAM */
  if (BSP_SDRAM_Init() != SDRAM_OK)
  {
    LOG_ERROR("SDRAM init failed");
    Error_Handler();
  }
#ifdef WITH_DOOM
  doom_memory_init();   /* the Doom engine's variables are in SDRAM */
#endif

  i2c_bus_init();
  player_start(INITIAL_VOLUME);

  if (xTaskCreate(gui_task, "gui", GUI_TASK_STACK, NULL, GUI_TASK_PRIORITY, NULL) != pdPASS)
  {
    Error_Handler();
  }

  vTaskStartScheduler();

  /* Only reached if there was not enough heap to start the scheduler */
  Error_Handler();
}

/* ---- GUI task -------------------------------------------------------------- */

static uint32_t lv_tick_cb(void)
{
  return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

static void lv_delay_cb(uint32_t ms)
{
  vTaskDelay(pdMS_TO_TICKS(ms));
}

static void lv_log_cb(lv_log_level_t level, const char *buf)
{
  (void)level;
  /* LVGL messages already end with a newline */
  LOG_WARN("lvgl: %.*s", (int)(strcspn(buf, "\r\n")), buf);
}

static void gui_task(void *arg)
{
  (void)arg;

  lv_init();
  lv_tick_set_cb(lv_tick_cb);
  lv_delay_set_cb(lv_delay_cb);
  lv_log_register_print_cb(lv_log_cb);
  lv_fs_sd_init();

  display_init();
  if (touch_init(xTaskGetCurrentTaskHandle()) == NULL)
  {
    LOG_ERROR("touch input unavailable");
  }
  jpeg_hw_init();
  video_init();
  shell_init();

  /* Draw the first frame now, then let the music start */
  lv_refr_now(NULL);
  player_ui_ready();

  for (;;)
  {
    touch_process();

    uint32_t sleep_ms = lv_timer_handler();
    if (sleep_ms > GUI_MAX_SLEEP_MS)
    {
      sleep_ms = GUI_MAX_SLEEP_MS;
    }
    if (touch_read_pending() && sleep_ms > TOUCH_PRESSED_POLL_MS)
    {
      sleep_ms = TOUCH_PRESSED_POLL_MS;
    }
    /* Sleep until LVGL's next timer is due or the touch interrupt wakes us */
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(sleep_ms > 0U ? sleep_ms : 1U));
  }
}

/* ---- system configuration -------------------------------------------------- */

/**
 * Memory attributes:
 *  - DTCM:        never cached (holds the DMA buffers, see DMA_BUFFER in main.h)
 *  - SRAM1/SRAM2: default map, write-back cacheable (task stacks, RTOS heap)
 *  - SDRAM:       all 16 MB write-through cacheable (frame buffers, LVGL heap,
 *                 video buffers); the rest of the bank area strongly ordered
 *  - QSPI area:   strongly ordered, no access (not used, avoids speculative reads)
 */
static void mpu_config(void)
{
  MPU_Region_InitTypeDef region = {0};

  HAL_MPU_Disable();

  region.Enable           = MPU_REGION_ENABLE;
  region.AccessPermission = MPU_REGION_FULL_ACCESS;
  region.DisableExec      = MPU_INSTRUCTION_ACCESS_ENABLE;
  region.SubRegionDisable = 0x00;

  region.Number       = MPU_REGION_NUMBER2;
  region.BaseAddress  = 0xC0000000;
  region.Size         = MPU_REGION_SIZE_256MB;
  region.TypeExtField = MPU_TEX_LEVEL0;
  region.IsCacheable  = MPU_ACCESS_NOT_CACHEABLE;
  region.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  region.IsShareable  = MPU_ACCESS_NOT_SHAREABLE;
  HAL_MPU_ConfigRegion(&region);

  region.Number       = MPU_REGION_NUMBER3;
  region.BaseAddress  = 0xC0000000;
  region.Size         = MPU_REGION_SIZE_16MB;
  region.IsCacheable  = MPU_ACCESS_CACHEABLE;   /* write-through (TEX 0, C=1, B=0) */
  HAL_MPU_ConfigRegion(&region);

  region.Number           = MPU_REGION_NUMBER4;
  region.BaseAddress      = 0x90000000;
  region.Size             = MPU_REGION_SIZE_256MB;
  region.AccessPermission = MPU_REGION_NO_ACCESS;
  region.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;
  region.DisableExec      = MPU_INSTRUCTION_ACCESS_DISABLE;
  HAL_MPU_ConfigRegion(&region);

  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

static void cpu_cache_enable(void)
{
  SCB_EnableICache();
  SCB_EnableDCache();
}

/**
 * SYSCLK 216 MHz from the 25 MHz HSE: PLLM 25, PLLN 432, PLLP 2.
 * PLLQ 9 gives the 48 MHz clock used by the SD card interface.
 * (PLLSAI is set up by the display driver, PLLI2S by the audio driver.)
 */
static void system_clock_config(void)
{
  RCC_OscInitTypeDef osc = {0};
  RCC_ClkInitTypeDef clk = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  osc.HSEState       = RCC_HSE_ON;
  osc.PLL.PLLState   = RCC_PLL_ON;
  osc.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
  osc.PLL.PLLM       = 25;
  osc.PLL.PLLN       = 432;
  osc.PLL.PLLP       = RCC_PLLP_DIV2;
  osc.PLL.PLLQ       = 9;
  if (HAL_RCC_OscConfig(&osc) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  clk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  clk.APB1CLKDivider = RCC_HCLK_DIV4;
  clk.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/* ---- fault handling -------------------------------------------------------- */

static void halt(void)
{
  __disable_irq();
  BSP_LED_On(LED_RED);
  /* Stop here if a debugger is attached so the call stack can be inspected */
  if ((CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) != 0U)
  {
    __BKPT(0);
  }
  for (;;)
  {
  }
}

void Error_Handler(void)
{
  halt();
}

/* Kept in globals so they can be read with a debugger after a halt */
volatile const char *g_assert_file;
volatile uint32_t    g_assert_line;
volatile const char *g_overflow_task;

void vAssertCalled(uint32_t line, const char *file)
{
  g_assert_file = file;
  g_assert_line = line;
  halt();
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *task_name)
{
  (void)task;
  g_overflow_task = task_name;
  halt();
}

/* ---- power / blocking waits ------------------------------------------------ */

/* Nothing to run: stop the core until the next interrupt (tick, DMA, touch...),
 * measuring the sleep time for the CPU load display */
void vApplicationIdleHook(void)
{
  sysmon_idle();
}

/**
 * HAL_Delay() is a busy-wait in the HAL. Drivers call it for hundreds of
 * milliseconds (e.g. the WM8994 start-up), which would keep the CPU spinning
 * and starve lower-priority tasks. Inside a task, sleep instead.
 */
void HAL_Delay(uint32_t ms)
{
  const bool in_isr = (__get_IPSR() != 0U);
  if (!in_isr && xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
  {
    /* +1 tick: like the HAL version, wait at least the requested time */
    vTaskDelay(pdMS_TO_TICKS(ms) + 1U);
    return;
  }

  const uint32_t start = HAL_GetTick();
  while ((HAL_GetTick() - start) < ms + 1U)
  {
  }
}

void vApplicationMallocFailedHook(void)
{
  halt();
}
