/**
 * @file  workspace.c
 * @brief The shared SDRAM area: one holder at a time, buffers carved off in turn.
 */
#include "workspace.h"

#include <stdint.h>

#include "log.h"

#define ALIGN 32U

static uint8_t          s_area[WORKSPACE_SIZE] __attribute__((section(".sdram"), aligned(ALIGN)));
static workspace_user_t s_user = WORKSPACE_NONE;
static size_t           s_used;

bool workspace_take(workspace_user_t user)
{
  if (s_user != WORKSPACE_NONE && s_user != user)
  {
    LOG_ERROR("workspace: wanted by %d, held by %d", (int)user, (int)s_user);
    return false;
  }
  s_user = user;
  s_used = 0;
  return true;
}

void *workspace_alloc(workspace_user_t user, size_t size)
{
  if (user != s_user || user == WORKSPACE_NONE)
  {
    LOG_ERROR("workspace: %d allocates without holding it", (int)user);
    return NULL;
  }
  const size_t start = (s_used + ALIGN - 1U) & ~(size_t)(ALIGN - 1U);
  if (size > WORKSPACE_SIZE || start > WORKSPACE_SIZE - size)
  {
    LOG_ERROR("workspace: %u bytes more do not fit", (unsigned)size);
    return NULL;
  }
  s_used = start + size;
  return &s_area[start];
}

void workspace_give(workspace_user_t user)
{
  if (s_user == user)
  {
    s_user = WORKSPACE_NONE;
    s_used = 0;
  }
}
