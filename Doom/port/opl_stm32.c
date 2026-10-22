//
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
//    OPL interface on the STM32F769 Discovery, after Chocolate Doom's
//    opl_sdl.c: the DOSBox OPL emulator (DBOPL) renders the music into the
//    sound task's mix (i_sound.c calls OPL_STM32_Render for each block),
//    and the music player's timed callbacks run there too, as the rendered
//    audio reaches their time. FreeRTOS mutexes take the place of SDL's.
//

#include <string.h>

#include "FreeRTOS.h"
#include "semphr.h"

#include "dbopl.h"
#include "opl.h"
#include "opl_internal.h"
#include "opl_queue.h"
#include "doom_port.h"   // after the engine's headers: they define false and true

typedef struct
{
    unsigned int rate;        // Number of times the timer is advanced per sec.
    unsigned int enabled;     // Non-zero if timer is enabled.
    unsigned int value;       // Last value that was set.
    uint64_t expire_time;     // Calculated time that timer will expire.
} opl_timer_t;

// When the callback mutex is held using OPL_Lock, callback functions
// are not invoked (recursive, like SDL's mutexes).

static SemaphoreHandle_t callback_mutex;

// Queue of callbacks waiting to be invoked, and the mutex for it.

static opl_callback_queue_t *callback_queue;
static SemaphoreHandle_t callback_queue_mutex;

// Current time, in us since startup (of rendered audio):

static uint64_t current_time;

// If non-zero, playback is currently paused.

static int opl_paused;

// Time offset (in us) due to the fact that callbacks
// were previously paused.

static uint64_t pause_offset;

// OPL software emulator structure.

static Chip opl_chip;
static int initialized;

// Register number that was written.

static int register_num = 0;

// Timers; DBOPL does not do timer stuff itself.

static opl_timer_t timer1 = { 12500, 0, 0, 0 };
static opl_timer_t timer2 = { 3125, 0, 0, 0 };

// Advance time by the specified number of samples, invoking any
// callback functions as appropriate.

static void AdvanceTime(unsigned int nsamples)
{
    opl_callback_t callback;
    void *callback_data;
    uint64_t us;

    xSemaphoreTake(callback_queue_mutex, portMAX_DELAY);

    us = ((uint64_t) nsamples * OPL_SECOND) / opl_sample_rate;
    current_time += us;

    if (opl_paused)
    {
        pause_offset += us;
    }

    // Keep invoking the callbacks that are due. The queue mutex must not
    // be held while one runs: it may schedule the next one.

    while (!OPL_Queue_IsEmpty(callback_queue)
        && current_time >= OPL_Queue_Peek(callback_queue) + pause_offset)
    {
        if (!OPL_Queue_Pop(callback_queue, &callback, &callback_data))
        {
            break;
        }

        xSemaphoreGive(callback_queue_mutex);
        xSemaphoreTakeRecursive(callback_mutex, portMAX_DELAY);
        callback(callback_data);
        xSemaphoreGiveRecursive(callback_mutex);
        xSemaphoreTake(callback_queue_mutex, portMAX_DELAY);
    }

    xSemaphoreGive(callback_queue_mutex);
}

// Render @nsamples mono samples of the music at opl_sample_rate into
// @out, running the callbacks that fall within them (sound task).

void OPL_STM32_Render(int32_t *out, unsigned int nsamples)
{
    unsigned int filled = 0;

    if (!initialized)
    {
        memset(out, 0, nsamples * sizeof(*out));
        return;
    }

    while (filled < nsamples)
    {
        uint64_t n;

        // Up to the next callback, so that it runs at the right sample

        xSemaphoreTake(callback_queue_mutex, portMAX_DELAY);
        if (opl_paused || OPL_Queue_IsEmpty(callback_queue))
        {
            n = nsamples - filled;
        }
        else
        {
            const uint64_t next = OPL_Queue_Peek(callback_queue) + pause_offset;
            n = (next > current_time) ? (next - current_time) * opl_sample_rate : 0;
            n = (n + OPL_SECOND - 1) / OPL_SECOND;
            if (n > nsamples - filled)
            {
                n = nsamples - filled;
            }
        }
        xSemaphoreGive(callback_queue_mutex);

        if (n > 0)
        {
            // Not while the game changes the registers
            xSemaphoreTakeRecursive(callback_mutex, portMAX_DELAY);
            Chip__GenerateBlock2(&opl_chip, (Bitu) n, out + filled);
            xSemaphoreGiveRecursive(callback_mutex);
        }
        filled += (unsigned int) n;

        AdvanceTime((unsigned int) n);
    }
}

static void OPL_STM32_Shutdown(void)
{
    if (initialized)
    {
        initialized = 0;
        OPL_Queue_Destroy(callback_queue);
        callback_queue = NULL;
    }
}

static int OPL_STM32_Init(unsigned int port_base)
{
    if (callback_mutex == NULL)
    {
        callback_mutex = xSemaphoreCreateRecursiveMutex();
        callback_queue_mutex = xSemaphoreCreateMutex();
        if (callback_mutex == NULL || callback_queue_mutex == NULL)
        {
            return 0;
        }
    }

    opl_paused = 0;
    pause_offset = 0;
    current_time = 0;

    callback_queue = OPL_Queue_Create();

    DBOPL_InitTables();
    Chip__Chip(&opl_chip);
    Chip__Setup(&opl_chip, opl_sample_rate);

    initialized = 1;
    return 1;
}

static unsigned int OPL_STM32_PortRead(opl_port_t port)
{
    unsigned int result = 0;

    if (port == OPL_REGISTER_PORT_OPL3)
    {
        return 0xff;
    }

    if (timer1.enabled && current_time > timer1.expire_time)
    {
        result |= 0x80;   // Either have expired
        result |= 0x40;   // Timer 1 has expired
    }

    if (timer2.enabled && current_time > timer2.expire_time)
    {
        result |= 0x80;   // Either have expired
        result |= 0x20;   // Timer 2 has expired
    }

    return result;
}

static void OPLTimer_CalculateEndTime(opl_timer_t *timer)
{
    int tics;

    // If the timer is enabled, calculate the time when the timer
    // will expire.

    if (timer->enabled)
    {
        tics = 0x100 - timer->value;
        timer->expire_time = current_time
                           + ((uint64_t) tics * OPL_SECOND) / timer->rate;
    }
}

static void WriteRegister(unsigned int reg_num, unsigned int value)
{
    switch (reg_num)
    {
        case OPL_REG_TIMER1:
            timer1.value = value;
            OPLTimer_CalculateEndTime(&timer1);
            break;

        case OPL_REG_TIMER2:
            timer2.value = value;
            OPLTimer_CalculateEndTime(&timer2);
            break;

        case OPL_REG_TIMER_CTRL:
            if (value & 0x80)
            {
                timer1.enabled = 0;
                timer2.enabled = 0;
            }
            else
            {
                if ((value & 0x40) == 0)
                {
                    timer1.enabled = (value & 0x01) != 0;
                    OPLTimer_CalculateEndTime(&timer1);
                }

                if ((value & 0x20) == 0)
                {
                    timer2.enabled = (value & 0x02) != 0;
                    OPLTimer_CalculateEndTime(&timer2);
                }
            }

            break;

        default:
            // OPL2 only: the music is rendered in mono (no OPL3 mode)
            Chip__WriteReg(&opl_chip, reg_num, value);
            break;
    }
}

static void OPL_STM32_PortWrite(opl_port_t port, unsigned int value)
{
    if (port == OPL_REGISTER_PORT)
    {
        register_num = value;
    }
    else if (port == OPL_REGISTER_PORT_OPL3)
    {
        register_num = value | 0x100;
    }
    else if (port == OPL_DATA_PORT)
    {
        WriteRegister(register_num, value);
    }
}

static void OPL_STM32_SetCallback(uint64_t us, opl_callback_t callback,
                                  void *data)
{
    xSemaphoreTake(callback_queue_mutex, portMAX_DELAY);
    OPL_Queue_Push(callback_queue, callback, data,
                   current_time - pause_offset + us);
    xSemaphoreGive(callback_queue_mutex);
}

static void OPL_STM32_ClearCallbacks(void)
{
    xSemaphoreTake(callback_queue_mutex, portMAX_DELAY);
    OPL_Queue_Clear(callback_queue);
    xSemaphoreGive(callback_queue_mutex);
}

static void OPL_STM32_Lock(void)
{
    xSemaphoreTakeRecursive(callback_mutex, portMAX_DELAY);
}

static void OPL_STM32_Unlock(void)
{
    xSemaphoreGiveRecursive(callback_mutex);
}

static void OPL_STM32_SetPaused(int paused)
{
    opl_paused = paused;
}

static void OPL_STM32_AdjustCallbacks(float factor)
{
    xSemaphoreTake(callback_queue_mutex, portMAX_DELAY);
    OPL_Queue_AdjustCallbacks(callback_queue, current_time, factor);
    xSemaphoreGive(callback_queue_mutex);
}

opl_driver_t opl_stm32_driver =
{
    "STM32",
    OPL_STM32_Init,
    OPL_STM32_Shutdown,
    OPL_STM32_PortRead,
    OPL_STM32_PortWrite,
    OPL_STM32_SetCallback,
    OPL_STM32_ClearCallbacks,
    OPL_STM32_Lock,
    OPL_STM32_Unlock,
    OPL_STM32_SetPaused,
    OPL_STM32_AdjustCallbacks,
};
