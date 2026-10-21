/**
 * @file  display.c
 * @brief OTM8009A 800x480 DSI panel in adapted command mode, as an LVGL display.
 *
 * How a frame reaches the glass:
 *  - LVGL renders directly into one of two RGB565 frame buffers in SDRAM
 *    (LV_DISPLAY_RENDER_MODE_DIRECT). The flush callback marks it "pending".
 *  - On the panel's tearing-effect (TE) interrupt, the pending buffer becomes
 *    the LTDC source and is pushed to the panel's own frame memory over DSI.
 *    LVGL may then render the next frame into the other buffer.
 *  - The panel is natively portrait, so the landscape image is sent in
 *    ZONES vertical stripes (one DSI refresh each) to avoid tearing.
 * The panel keeps the last image, so nothing is sent while the UI is idle.
 * Panel commands (brightness, display on/off) are sent by the GUI task between
 * frame transfers: while one is being sent, the TE interrupt starts no transfer.
 * Video mode: LVGL is paused and the video player writes its frames straight
 * into the two frame buffers, handing each finished one to the TE interrupt.
 */
#include "display.h"

#include <string.h>

#include "main.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "otm8009a/otm8009a.h"

#define ZONES        4
#define ZONE_WIDTH   (DISPLAY_WIDTH / ZONES)
#define BYTES_PER_PX 2U

/* LTDC timings for command mode: only the active area matters */
#define HSYNC 1
#define HBP   1
#define HFP   1
#define VSYNC 1
#define VBP   1
#define VFP   1

/* Scan line at which the panel raises TE (tuned by ST for 4 zones) */
#define TE_SCANLINE 283
/* DCS "set tear scanline" (not defined by the OTM8009A component driver) */
#define DCS_SET_TEAR_SCANLINE 0x44

/* LCD reset line */
#define LCD_RESET_PORT GPIOJ
#define LCD_RESET_PIN  GPIO_PIN_15

/* DSI TE input */
#define LCD_TE_PORT    GPIOJ
#define LCD_TE_PIN     GPIO_PIN_2

#define DSI_IRQ_PRIORITY 3

static LTDC_HandleTypeDef s_ltdc;
static DSI_HandleTypeDef  s_dsi;
static lv_display_t      *s_disp;

/* Two full-screen frame buffers in external SDRAM (see the .sdram section) */
static uint16_t s_fb[2][DISPLAY_WIDTH * DISPLAY_HEIGHT]
  __attribute__((section(".sdram"), aligned(64)));

static uint16_t *volatile s_pending_fb;  /* rendered by LVGL, waiting for TE */
static uint16_t *volatile s_active_fb;   /* currently sent to / shown by the panel */
static uint16_t          *s_lvgl_last;   /* last buffer LVGL flushed: it draws into the other next */
static volatile bool      s_release_lvgl;/* pending buffer came from a flush */
static SemaphoreHandle_t  s_flush_done;  /* given when LVGL may draw again */

/* Upper bound for one wait (normally released within one refresh, ~17-33 ms) */
#define FLUSH_WAIT_TIMEOUT_MS 100
static volatile bool      s_refreshing;  /* a multi-zone DSI transfer is running */
static volatile bool      s_cmd_busy;    /* a panel command is being sent: no transfer starts */
static bool               s_on = true;
static volatile bool      s_video;       /* video mode: frames come from display_video_present() */
static uint8_t            s_brightness = 0x7F;   /* as set by OTM8009A_Init() */

/* DCS commands (MIPI Display Command Set) */
#define DCS_DISPLAY_OFF        0x28U
#define DCS_DISPLAY_ON         0x29U
#define DCS_WRITE_BRIGHTNESS   0x51U
static volatile uint32_t  s_next_zone;

/* CASET parameters (start/end column, big endian) for each stripe */
static uint8_t s_zone_cols[ZONES][4] =
{
  {0x00, 0x00, 0x00, 0xC7}, /*   0 -> 199 */
  {0x00, 0xC8, 0x01, 0x8F}, /* 200 -> 399 */
  {0x01, 0x90, 0x02, 0x57}, /* 400 -> 599 */
  {0x02, 0x58, 0x03, 0x1F}, /* 600 -> 799 */
};
static uint8_t s_page_rows[4] = {0x00, 0x00, 0x01, 0xDF}; /* 0 -> 479 */

/* Used by the OTM8009A component driver to send its init sequence */
void DSI_IO_WriteCmd(uint32_t NbrParams, uint8_t *pParams)
{
  if (NbrParams <= 1U)
  {
    HAL_DSI_ShortWrite(&s_dsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, pParams[0], pParams[1]);
  }
  else
  {
    HAL_DSI_LongWrite(&s_dsi, 0, DSI_DCS_LONG_PKT_WRITE, NbrParams, pParams[NbrParams], pParams);
  }
}

static void panel_reset(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOJ_CLK_ENABLE();
  gpio.Pin   = LCD_RESET_PIN;
  gpio.Mode  = GPIO_MODE_OUTPUT_PP;
  gpio.Pull  = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LCD_RESET_PORT, &gpio);

  HAL_GPIO_WritePin(LCD_RESET_PORT, LCD_RESET_PIN, GPIO_PIN_RESET);
  HAL_Delay(20);
  HAL_GPIO_WritePin(LCD_RESET_PORT, LCD_RESET_PIN, GPIO_PIN_SET);
  HAL_Delay(10);
}

static void peripherals_reset_and_clock(void)
{
  RCC_PeriphCLKInitTypeDef clk = {0};

  __HAL_RCC_LTDC_CLK_ENABLE();
  __HAL_RCC_LTDC_FORCE_RESET();
  __HAL_RCC_LTDC_RELEASE_RESET();

  __HAL_RCC_DSI_CLK_ENABLE();
  __HAL_RCC_DSI_FORCE_RESET();
  __HAL_RCC_DSI_RELEASE_RESET();

  /* LTDC pixel clock: PLLSAI VCO 280 MHz / R 7 / DIVR 2 = 20 MHz
   * (HSE 25 MHz / PLLM 25 = 1 MHz VCO input) */
  clk.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
  clk.PLLSAI.PLLSAIN       = 280;
  clk.PLLSAI.PLLSAIR       = 7;
  clk.PLLSAIDivR           = RCC_PLLSAIDIVR_2;
  if (HAL_RCCEx_PeriphCLKConfig(&clk) != HAL_OK)
  {
    Error_Handler();
  }
}

static void dsi_set_low_power_commands(bool enable)
{
  DSI_LPCmdTypeDef lp;
  const uint32_t on = enable ? 1U : 0U;

  lp.LPGenShortWriteNoP  = on ? DSI_LP_GSW0P_ENABLE : DSI_LP_GSW0P_DISABLE;
  lp.LPGenShortWriteOneP = on ? DSI_LP_GSW1P_ENABLE : DSI_LP_GSW1P_DISABLE;
  lp.LPGenShortWriteTwoP = on ? DSI_LP_GSW2P_ENABLE : DSI_LP_GSW2P_DISABLE;
  lp.LPGenShortReadNoP   = on ? DSI_LP_GSR0P_ENABLE : DSI_LP_GSR0P_DISABLE;
  lp.LPGenShortReadOneP  = on ? DSI_LP_GSR1P_ENABLE : DSI_LP_GSR1P_DISABLE;
  lp.LPGenShortReadTwoP  = on ? DSI_LP_GSR2P_ENABLE : DSI_LP_GSR2P_DISABLE;
  lp.LPGenLongWrite      = on ? DSI_LP_GLW_ENABLE   : DSI_LP_GLW_DISABLE;
  lp.LPDcsShortWriteNoP  = on ? DSI_LP_DSW0P_ENABLE : DSI_LP_DSW0P_DISABLE;
  lp.LPDcsShortWriteOneP = on ? DSI_LP_DSW1P_ENABLE : DSI_LP_DSW1P_DISABLE;
  lp.LPDcsShortReadNoP   = on ? DSI_LP_DSR0P_ENABLE : DSI_LP_DSR0P_DISABLE;
  lp.LPDcsLongWrite      = on ? DSI_LP_DLW_ENABLE   : DSI_LP_DLW_DISABLE;
  HAL_DSI_ConfigCommand(&s_dsi, &lp);
}

static void dsi_init(void)
{
  DSI_PLLInitTypeDef   pll = {0};
  DSI_CmdCfgTypeDef    cmd = {0};

  s_dsi.Instance = DSI;
  HAL_DSI_DeInit(&s_dsi);

  /* Lane byte clock: 25 MHz / IDF 5 * 2 * NDIV 100 / ODF 1 / 8 = 62.5 MHz (500 Mbit/s per lane) */
  pll.PLLNDIV = 100;
  pll.PLLIDF  = DSI_PLL_IN_DIV5;
  pll.PLLODF  = DSI_PLL_OUT_DIV1;
  s_dsi.Init.NumberOfLanes = DSI_TWO_DATA_LANES;
  s_dsi.Init.TXEscapeCkdiv = 0x4;
  if (HAL_DSI_Init(&s_dsi, &pll) != HAL_OK)
  {
    Error_Handler();
  }

  cmd.VirtualChannelID      = 0;
  cmd.HSPolarity            = DSI_HSYNC_ACTIVE_HIGH;
  cmd.VSPolarity            = DSI_VSYNC_ACTIVE_HIGH;
  cmd.DEPolarity            = DSI_DATA_ENABLE_ACTIVE_HIGH;
  cmd.ColorCoding           = DSI_RGB565;
  cmd.CommandSize           = ZONE_WIDTH;
  cmd.TearingEffectSource   = DSI_TE_EXTERNAL;
  cmd.TearingEffectPolarity = DSI_TE_RISING_EDGE;
  cmd.VSyncPol              = DSI_VSYNC_FALLING;
  cmd.AutomaticRefresh      = DSI_AR_DISABLE;
  cmd.TEAcknowledgeRequest  = DSI_TE_ACKNOWLEDGE_ENABLE;
  HAL_DSI_ConfigAdaptedCommandMode(&s_dsi, &cmd);

  /* Panel configuration is sent in low-power mode */
  dsi_set_low_power_commands(true);
}

static void dsi_phy_timing_init(void)
{
  DSI_PHY_TimerTypeDef phy = {0};

  phy.ClockLaneHS2LPTime  = 35;
  phy.ClockLaneLP2HSTime  = 35;
  phy.DataLaneHS2LPTime   = 35;
  phy.DataLaneLP2HSTime   = 35;
  phy.DataLaneMaxReadTime = 0;
  phy.StopWaitTime        = 10;
  HAL_DSI_ConfigPhyTimer(&s_dsi, &phy);
}

static void ltdc_init(void)
{
  /* No HAL_LTDC_DeInit(): the LTDC was just reset in peripherals_reset_and_clock(),
   * and DeInit would also gate its clock through HAL_LTDC_MspDeInit(). */
  s_ltdc.Instance = LTDC;

  /* The LTDC only produces one ZONE_WIDTH-wide stripe at a time */
  s_ltdc.Init.HorizontalSync     = HSYNC;
  s_ltdc.Init.VerticalSync       = VSYNC;
  s_ltdc.Init.AccumulatedHBP     = HSYNC + HBP;
  s_ltdc.Init.AccumulatedVBP     = VSYNC + VBP;
  s_ltdc.Init.AccumulatedActiveW = HSYNC + HBP + ZONE_WIDTH;
  s_ltdc.Init.AccumulatedActiveH = VSYNC + VBP + DISPLAY_HEIGHT;
  s_ltdc.Init.TotalWidth         = HSYNC + HBP + ZONE_WIDTH + HFP;
  s_ltdc.Init.TotalHeigh         = VSYNC + VBP + DISPLAY_HEIGHT + VFP;
  s_ltdc.Init.HSPolarity         = LTDC_HSPOLARITY_AL;
  s_ltdc.Init.VSPolarity         = LTDC_VSPOLARITY_AL;
  s_ltdc.Init.DEPolarity         = LTDC_DEPOLARITY_AL;
  s_ltdc.Init.PCPolarity         = LTDC_PCPOLARITY_IPC;
  if (HAL_LTDC_Init(&s_ltdc) != HAL_OK)
  {
    Error_Handler();
  }
}

static void ltdc_layer_init(void)
{
  LTDC_LayerCfgTypeDef layer = {0};

  layer.WindowX0        = 0;
  layer.WindowX1        = ZONE_WIDTH;
  layer.WindowY0        = 0;
  layer.WindowY1        = DISPLAY_HEIGHT;
  layer.PixelFormat     = LTDC_PIXEL_FORMAT_RGB565;
  layer.FBStartAdress   = (uint32_t)s_active_fb;
  layer.Alpha           = 255;
  layer.Alpha0          = 0;
  layer.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
  layer.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
  layer.ImageWidth      = ZONE_WIDTH;
  layer.ImageHeight     = DISPLAY_HEIGHT;
  HAL_LTDC_ConfigLayer(&s_ltdc, &layer, 0);

  /* ...but each stripe is read out of a full-width frame buffer */
  HAL_LTDC_SetPitch(&s_ltdc, DISPLAY_WIDTH, 0);
}

static void te_pin_init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOJ_CLK_ENABLE();
  gpio.Pin       = LCD_TE_PIN;
  gpio.Mode      = GPIO_MODE_AF_PP;
  gpio.Pull      = GPIO_NOPULL;
  gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
  gpio.Alternate = GPIO_AF13_DSI;
  HAL_GPIO_Init(LCD_TE_PORT, &gpio);
}

static void panel_set_columns(uint32_t zone)
{
  HAL_DSI_LongWrite(&s_dsi, 0, DSI_DCS_LONG_PKT_WRITE, 4, OTM8009A_CMD_CASET, s_zone_cols[zone]);
}

static void panel_enable_tearing_effect(void)
{
  static uint8_t scanline[2] = {TE_SCANLINE >> 8, TE_SCANLINE & 0xFF};

  HAL_DSI_LongWrite(&s_dsi, 0, DSI_DCS_LONG_PKT_WRITE, 2, DCS_SET_TEAR_SCANLINE, scanline);
  HAL_DSI_ShortWrite(&s_dsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, OTM8009A_CMD_TEEON, 0x00);
}

/* Point the LTDC at a given stripe of the active frame buffer */
static void ltdc_select_zone(uint32_t zone)
{
  __HAL_DSI_WRAPPER_DISABLE(&s_dsi);
  LTDC_LAYER(&s_ltdc, 0)->CFBAR = (uint32_t)s_active_fb + zone * ZONE_WIDTH * BYTES_PER_PX;
  __HAL_LTDC_RELOAD_CONFIG(&s_ltdc);
  __HAL_DSI_WRAPPER_ENABLE(&s_dsi);
}

/* ---- LVGL glue ------------------------------------------------------------ */

/* LVGL would otherwise spin in a busy loop until the frame buffer is released:
 * sleep instead until the TE interrupt hands the buffer back. */
static void flush_wait_cb(lv_display_t *disp)
{
  (void)disp;
  while (s_release_lvgl)
  {
    xSemaphoreTake(s_flush_done, pdMS_TO_TICKS(FLUSH_WAIT_TIMEOUT_MS));
  }
}

static void flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
  (void)area;

  if (!lv_display_flush_is_last(disp))
  {
    /* Direct mode: LVGL already drew into the frame buffer; wait for the last area */
    lv_display_flush_ready(disp);
    return;
  }

  /* Make sure all pixel writes reached SDRAM before the DSI transfer reads them */
  __DSB();

  /* flush_ready is signalled from the TE interrupt once this buffer is on its way */
  __disable_irq();
  s_lvgl_last    = (uint16_t *)px_map;
  s_pending_fb   = (uint16_t *)px_map;
  s_release_lvgl = true;
  __enable_irq();
}

lv_display_t *display_init(void)
{
  memset(s_fb, 0, sizeof(s_fb));
  s_active_fb  = s_fb[0];
  s_pending_fb = NULL;

  /* Same sequence as ST's STM32F769I-Discovery command-mode examples */
  panel_reset();
  peripherals_reset_and_clock();
  dsi_init();
  ltdc_init();
  /* The layer must be configured before the DSI host starts: once the DSI
   * wrapper drives the LTDC in command mode, LTDC register writes are ignored
   * (except while the wrapper is disabled, see ltdc_select_zone()). */
  ltdc_layer_init();
  HAL_DSI_Start(&s_dsi);

  HAL_DSI_ShortWrite(&s_dsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, OTM8009A_CMD_DISPOFF, 0x00);
  dsi_phy_timing_init();
  OTM8009A_Init(OTM8009A_FORMAT_RBG565, OTM8009A_ORIENTATION_LANDSCAPE);

  /* Frame data goes in high-speed mode */
  dsi_set_low_power_commands(false);
  HAL_DSI_ConfigFlowControl(&s_dsi, DSI_FLOW_CONTROL_BTA);
  te_pin_init();

  panel_set_columns(0);
  HAL_DSI_LongWrite(&s_dsi, 0, DSI_DCS_LONG_PKT_WRITE, 4, OTM8009A_CMD_PASET, s_page_rows);

  HAL_NVIC_SetPriority(DSI_IRQn, DSI_IRQ_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(DSI_IRQn);

  HAL_Delay(50);
  panel_enable_tearing_effect();

  /* Push the (black) first buffer on the next TE, then switch the panel on */
  s_pending_fb = s_fb[0];
  HAL_DSI_ShortWrite(&s_dsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, OTM8009A_CMD_DISPON, 0x00);

  s_disp = lv_display_create(DISPLAY_WIDTH, DISPLAY_HEIGHT);
  lv_display_set_color_format(s_disp, LV_COLOR_FORMAT_RGB565);
  lv_display_set_buffers(s_disp, s_fb[0], s_fb[1], sizeof(s_fb[0]), LV_DISPLAY_RENDER_MODE_DIRECT);
  lv_display_set_flush_cb(s_disp, flush_cb);
  s_flush_done = xSemaphoreCreateBinary();
  configASSERT(s_flush_done != NULL);
  lv_display_set_flush_wait_cb(s_disp, flush_wait_cb);
  return s_disp;
}

/* ---- panel commands ----------------------------------------------------- */

/* Send a DCS command from the GUI task, between frame transfers */
static void send_panel_command(uint8_t cmd, bool has_param, uint8_t param)
{
  for (;;)
  {
    taskENTER_CRITICAL();             /* masks the DSI interrupt */
    const bool idle = !s_refreshing;
    if (idle)
    {
      s_cmd_busy = true;
    }
    taskEXIT_CRITICAL();
    if (idle)
    {
      break;
    }
    vTaskDelay(1);                     /* a transfer takes ~17 ms */
  }
  if (has_param)
  {
    HAL_DSI_ShortWrite(&s_dsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, cmd, param);
  }
  else
  {
    HAL_DSI_ShortWrite(&s_dsi, 0, DSI_DCS_SHORT_PKT_WRITE_P0, cmd, 0x00);
  }
  s_cmd_busy = false;
}

void display_set_brightness(uint8_t percent)
{
  if (percent > 100U)
  {
    percent = 100U;
  }
  s_brightness = (uint8_t)((percent * 255U + 50U) / 100U);
  if (s_on)
  {
    send_panel_command(DCS_WRITE_BRIGHTNESS, true, s_brightness);
  }
}

void display_set_on(bool on)
{
  if (on == s_on)
  {
    return;
  }
  s_on = on;
  if (on)
  {
    send_panel_command(DCS_DISPLAY_ON, false, 0);
    send_panel_command(DCS_WRITE_BRIGHTNESS, true, s_brightness);
    /* Nothing was drawn while off: draw everything again (in video mode,
     * display_video_end() does that) */
    if (!s_video)
    {
      lv_display_enable_invalidation(s_disp, true);
      lv_obj_invalidate(lv_screen_active());
    }
  }
  else
  {
    send_panel_command(DCS_WRITE_BRIGHTNESS, true, 0);
    send_panel_command(DCS_DISPLAY_OFF, false, 0);
    if (!s_video)
    {
      lv_display_enable_invalidation(s_disp, false);   /* no rendering for a dark screen */
    }
  }
}

bool display_is_on(void)
{
  return s_on;
}

/* ---- video mode ---------------------------------------------------------- */

void display_video_begin(void)
{
  /* What LVGL has not drawn yet is drawn now, then it draws nothing (its
   * invalidation is off: here unless the screen is, see display_set_on); its
   * last frame may still be on its way */
  lv_refr_now(s_disp);
  if (s_on)
  {
    lv_display_enable_invalidation(s_disp, false);
  }
  while (s_release_lvgl || s_pending_fb != NULL)
  {
    vTaskDelay(1);
  }
  s_video = true;
}

uint16_t *display_video_frame(void)
{
  /* The buffer that is neither on screen nor waiting to be: wait while the
   * previous frame is still pending (it becomes the one on screen) */
  while (s_pending_fb != NULL)
  {
    vTaskDelay(1);
  }
  return (s_active_fb == s_fb[0]) ? s_fb[1] : s_fb[0];
}

void display_video_present(uint16_t *frame)
{
  __DSB();                 /* all pixel writes reach SDRAM before the transfer */
  s_pending_fb = frame;    /* sent on the next TE */
}

void display_video_end(void)
{
  while (s_pending_fb != NULL)
  {
    vTaskDelay(1);
  }
  /* LVGL draws next into the buffer it did not flush last: if that is the one
   * on screen, the last frame moves to the other one first */
  if (s_active_fb != s_lvgl_last)
  {
    uint16_t *other = (s_active_fb == s_fb[0]) ? s_fb[1] : s_fb[0];
    memcpy(other, (const uint16_t *)s_active_fb, sizeof(s_fb[0]));
    display_video_present(other);
    while (s_pending_fb != NULL)
    {
      vTaskDelay(1);
    }
  }
  s_video = false;
  /* Both buffers hold video: LVGL draws the whole screen again */
  if (s_on)
  {
    lv_display_enable_invalidation(s_disp, true);
  }
  lv_obj_invalidate(lv_screen_active());
}

void display_dsi_irq_handler(void)
{
  HAL_DSI_IRQHandler(&s_dsi);
}

/* ---- DSI callbacks (interrupt context) ------------------------------------ */

void HAL_DSI_TearingEffectCallback(DSI_HandleTypeDef *hdsi)
{
  if (s_refreshing || s_cmd_busy || s_pending_fb == NULL)
  {
    return;  /* transfer or panel command in progress, or nothing new to show */
  }

  s_active_fb  = s_pending_fb;
  s_pending_fb = NULL;
  ltdc_select_zone(0);

  s_refreshing = true;
  s_next_zone  = 1;
  HAL_DSI_Refresh(hdsi);

  /* The previous buffer is no longer read: LVGL may draw into it */
  if (s_release_lvgl)
  {
    s_release_lvgl = false;
    lv_display_flush_ready(s_disp);
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(s_flush_done, &woken);
    portYIELD_FROM_ISR(woken);
  }
}

void HAL_DSI_EndOfRefreshCallback(DSI_HandleTypeDef *hdsi)
{
  if (s_next_zone < ZONES)
  {
    ltdc_select_zone(s_next_zone);
    panel_set_columns(s_next_zone);
    s_next_zone++;
    HAL_DSI_Refresh(hdsi);
  }
  else
  {
    /* Frame complete: rewind to the first stripe for the next one */
    ltdc_select_zone(0);
    panel_set_columns(0);
    s_refreshing = false;
  }
}
