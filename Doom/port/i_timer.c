//
// Copyright(C) 1993-1996 Id Software, Inc.
// Copyright(C) 2005-2014 Simon Howard
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// DESCRIPTION:
//    Timer functions, on the FreeRTOS tick (1 ms). The time stands still
//    while the game is paused (the app left), so that no tics pile up.
//

#include "FreeRTOS.h"
#include "task.h"

#include "i_timer.h"
#include "doomtype.h"
#include "doom_port.h"   // after the engine's headers: they define false and true

static TickType_t basetime;
static TickType_t paused_ticks;     // time spent paused, not counted

void I_TimerPaused(uint32_t ticks)
{
    paused_ticks += ticks;
}

static uint32_t ElapsedMS(void)
{
    const TickType_t now = xTaskGetTickCount();

    if (basetime == 0)
    {
        basetime = now;
    }

    return (uint32_t) ((now - basetime - paused_ticks) * portTICK_PERIOD_MS);
}

// returns time in 1/35th second tics

int I_GetTime (void)
{
    return (int) (((uint64_t) ElapsedMS() * TICRATE) / 1000);
}

// Same as I_GetTime, but returns time in milliseconds

int I_GetTimeMS(void)
{
    return (int) ElapsedMS();
}

// Sleep for a specified number of ms

void I_Sleep(int ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms > 0 ? ms : 1));
}

void I_WaitVBL(int count)
{
    I_Sleep((count * 1000) / 70);
}

void I_InitTimer(void)
{
}
