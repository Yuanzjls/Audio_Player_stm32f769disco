/**
 * @file  app_sysinfo.c
 * @brief System info app: firmware, uptime, CPU load, memory, SD card, and
 *        the FreeRTOS tasks with their free stack. Updated once a second
 *        while on screen.
 */
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "player.h"
#include "shell.h"
#include "sysmon.h"
#include "theme.h"

#define REFRESH_MS   1000
#define PAD          32
#define COL_W        (800 / 2 - PAD - PAD / 2)
#define MAX_TASKS    12
#define NAME_W       150          /* the value starts after the name column */

typedef struct
{
  lv_obj_t *value;
  lv_obj_t *bar;         /* NULL for plain rows */
} row_t;

static row_t      s_version, s_uptime, s_cpu, s_heap, s_gui, s_card, s_tracks;
static lv_obj_t  *s_tasks;
static lv_timer_t *s_timer;

static lv_obj_t *make_label(lv_obj_t *parent, const lv_font_t *font, lv_color_t color)
{
  lv_obj_t *label = lv_label_create(parent);
  lv_obj_set_style_text_font(label, font, 0);
  lv_obj_set_style_text_color(label, color, 0);
  lv_label_set_text(label, "");
  return label;
}

/* "Name ........ value", optionally with a bar below */
static row_t make_row(lv_obj_t *parent, int32_t x, int32_t y, const char *name, bool with_bar)
{
  row_t r = { NULL, NULL };
  lv_obj_t *label = make_label(parent, &lv_font_montserrat_20, COLOR_TEXT_DIM);
  lv_label_set_text(label, name);
  lv_obj_set_pos(label, x, y);

  /* One line, right-aligned; a value too long for it ends in "..." */
  r.value = make_label(parent, &lv_font_montserrat_20, COLOR_TEXT);
  lv_label_set_long_mode(r.value, LV_LABEL_LONG_MODE_DOTS);
  lv_obj_set_size(r.value, COL_W - NAME_W, lv_font_get_line_height(&lv_font_montserrat_20));
  lv_obj_set_style_text_align(r.value, LV_TEXT_ALIGN_RIGHT, 0);
  lv_obj_set_pos(r.value, x + NAME_W, y);

  if (with_bar)
  {
    r.bar = lv_bar_create(parent);
    lv_obj_set_size(r.bar, COL_W, 10);
    lv_obj_set_pos(r.bar, x, y + 32);
    lv_bar_set_range(r.bar, 0, 100);
    lv_obj_set_style_bg_color(r.bar, COLOR_SURFACE, LV_PART_MAIN);
    lv_obj_set_style_bg_color(r.bar, COLOR_ACCENT, LV_PART_INDICATOR);
  }
  return r;
}

static void set_percent(const row_t *r, uint32_t used, uint32_t total, const char *unit)
{
  const uint32_t pct = (total != 0U) ? (uint32_t)(((uint64_t)used * 100U) / total) : 0U;
  lv_label_set_text_fmt(r->value, "%lu / %lu %s  (%lu%%)", (unsigned long)used, (unsigned long)total, unit,
                        (unsigned long)pct);
  lv_bar_set_value(r->bar, (int32_t)pct, LV_ANIM_OFF);
}

static const char *task_state(eTaskState s)
{
  switch (s)
  {
    case eRunning:   return "running";
    case eReady:     return "ready";
    case eBlocked:   return "blocked";
    case eSuspended: return "suspended";
    default:         return "-";
  }
}

static void refresh(lv_timer_t *timer)
{
  (void)timer;
  player_status_t st;
  player_get_status(&st);

  const uint32_t up = xTaskGetTickCount() / configTICK_RATE_HZ;
  lv_label_set_text_fmt(s_uptime.value, "%lu:%02lu:%02lu", (unsigned long)(up / 3600U),
                        (unsigned long)(up / 60U % 60U), (unsigned long)(up % 60U));

  const uint32_t cpu = shell_cpu_load();
  lv_label_set_text_fmt(s_cpu.value, "%lu %%", (unsigned long)cpu);
  lv_bar_set_value(s_cpu.bar, (int32_t)cpu, LV_ANIM_OFF);

  uint32_t heap_used, heap_total;
  sysmon_rtos_heap(&heap_used, &heap_total);
  set_percent(&s_heap, heap_used / 1024U, heap_total / 1024U, "KB");

  lv_mem_monitor_t mon;
  lv_mem_monitor(&mon);
  set_percent(&s_gui, (uint32_t)((mon.total_size - mon.free_size) / 1024U), (uint32_t)(mon.total_size / 1024U), "KB");

  if (st.card_mb != 0U)
  {
    lv_label_set_text_fmt(s_card.value, "%s, %lu.%lu GB", st.card_fs, (unsigned long)(st.card_mb / 1024U),
                          (unsigned long)((st.card_mb % 1024U) * 10U / 1024U));
  }
  else
  {
    lv_label_set_text(s_card.value, "none");
  }
  lv_label_set_text_fmt(s_tracks.value, "%u", (unsigned)st.track_count);

  /* Tasks: name, priority, state, free stack */
  TaskStatus_t tasks[MAX_TASKS];
  const UBaseType_t n = uxTaskGetSystemState(tasks, MAX_TASKS, NULL);
  lv_table_set_row_count(s_tasks, (uint32_t)n + 1U);
  for (UBaseType_t i = 0; i < n; i++)
  {
    lv_table_set_cell_value(s_tasks, i + 1U, 0, tasks[i].pcTaskName);
    lv_table_set_cell_value_fmt(s_tasks, i + 1U, 1, "%lu", (unsigned long)tasks[i].uxCurrentPriority);
    lv_table_set_cell_value(s_tasks, i + 1U, 2, task_state(tasks[i].eCurrentState));
    lv_table_set_cell_value_fmt(s_tasks, i + 1U, 3, "%lu B",
                                (unsigned long)(tasks[i].usStackHighWaterMark * sizeof(StackType_t)));
  }
}

static void sysinfo_create(lv_obj_t *screen)
{
  lv_obj_t *c = shell_app_header(screen, "System info");
  const int32_t left  = PAD;
  const int32_t right = 800 / 2 + PAD / 2;

  s_version = make_row(c, left, 24, "Firmware", false);
  lv_label_set_text(s_version.value, firmware_version());
  s_uptime = make_row(c, left, 64, "Uptime", false);
  s_card   = make_row(c, left, 104, "SD card", false);
  s_tracks = make_row(c, left, 144, "Songs", false);
  row_t chip = make_row(c, left, 184, "Chip", false);
  lv_label_set_text_fmt(chip.value, "STM32F769 at %lu MHz", (unsigned long)(SystemCoreClock / 1000000U));

  s_cpu  = make_row(c, right, 24, "CPU", true);
  s_heap = make_row(c, right, 90, "RTOS heap", true);
  s_gui  = make_row(c, right, 156, "GUI memory", true);

  /* Task table across the bottom */
  s_tasks = lv_table_create(c);
  lv_obj_set_pos(s_tasks, left, 236);
  lv_obj_set_size(s_tasks, 800 - 2 * PAD, 480 - 64 - 236 - 16);
  lv_table_set_column_count(s_tasks, 4);
  static const int32_t widths[4] = { 220, 140, 180, 190 };
  static const char *const heads[4] = { "Task", "Priority", "State", "Free stack" };
  for (uint32_t i = 0; i < 4U; i++)
  {
    lv_table_set_column_width(s_tasks, i, widths[i]);
    lv_table_set_cell_value(s_tasks, 0, i, heads[i]);
  }
  lv_obj_set_style_text_font(s_tasks, &lv_font_montserrat_14, 0);
  lv_obj_set_style_pad_ver(s_tasks, 6, LV_PART_ITEMS);
  lv_obj_set_style_border_width(s_tasks, 0, 0);

  s_timer = lv_timer_create(refresh, REFRESH_MS, NULL);
  lv_timer_pause(s_timer);
}

static void sysinfo_show(void)
{
  refresh(NULL);
  lv_timer_resume(s_timer);
}

static void sysinfo_hide(void)
{
  lv_timer_pause(s_timer);
}

const app_t app_sysinfo = {
  .name   = "System info",
  .icon   = "i",
  .color  = 0x0F9D58,
  .create = sysinfo_create,
  .show   = sysinfo_show,
  .hide   = sysinfo_hide,
};
