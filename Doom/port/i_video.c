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
//    The video interface on the STM32F769 Discovery: the 320 x 200 picture,
//    doubled to 640 x 400, goes straight into the display's frame buffers
//    (doom_port.c hands them out and puts the buttons around it). Input
//    comes from the touch screen (doom_port_input).
//

#include <limits.h>
#include <stdint.h>
#include <string.h>

#include "config.h"
#include "d_event.h"
#include "doomkeys.h"
#include "i_video.h"
#include "m_controls.h"
#include "tables.h"
#include "v_video.h"
#include "z_zone.h"
#include "doom_port.h"   // after the engine's headers: they define false and true

// The screen buffer; this is modified to draw things to the screen

byte *I_VideoBuffer = NULL;

// If true, game is running as a screensaver

boolean screensaver_mode = false;

// Flag indicating whether the screen is currently visible:
// when the screen isnt visible, don't render the screen

boolean screenvisible;

// Mouse acceleration (the touch screen's drags come in as mouse motion)

float mouse_acceleration = 2.0;
int mouse_threshold = 10;

// Gamma correction level to use

int usegamma = 0;

int usemouse = 1;

// If true, keyboard mapping is ignored, like in Vanilla Doom.

int vanilla_keyboard_mapping = true;

// Palette converted to RGB565, each colour twice (two pixels per word)

static uint32_t palette2[256];

void I_InitGraphics (void)
{
    I_VideoBuffer = (byte *) Z_Malloc (SCREENWIDTH * SCREENHEIGHT, PU_STATIC, NULL);
    memset(I_VideoBuffer, 0, SCREENWIDTH * SCREENHEIGHT);

    // The touch controls: always run, and a key for the next weapon
    // (called after the configuration is loaded, so these win)
    joybspeed = 29;
    if (key_nextweapon == 0)
    {
        key_nextweapon = ']';
    }

    screenvisible = true;
}

void I_ShutdownGraphics (void)
{
    Z_Free (I_VideoBuffer);
}

void I_StartFrame (void)
{
}

void I_StartTic (void)
{
    doom_port_input();
}

void I_UpdateNoBlit (void)
{
}

void I_FinishUpdate (void)
{
    doom_port_checkpoint();

    uint16_t *frame = doom_port_frame();
    const byte *src = I_VideoBuffer;

    for (int y = 0; y < SCREENHEIGHT; y++)
    {
        uint32_t *row0 = (uint32_t *) &frame[(DOOM_GAME_Y + 2 * y) * DOOM_SCREEN_W + DOOM_GAME_X];
        uint32_t *row1 = row0 + DOOM_SCREEN_W / 2;

        for (int x = 0; x < SCREENWIDTH; x++)
        {
            const uint32_t two = palette2[*src++];
            row0[x] = two;
            row1[x] = two;
        }
    }

    doom_port_present(frame);
}

//
// I_ReadScreen
//
void I_ReadScreen (byte* scr)
{
    memcpy (scr, I_VideoBuffer, SCREENWIDTH * SCREENHEIGHT);
}

//
// I_SetPalette
//
void I_SetPalette (byte* palette)
{
    for (int i = 0; i < 256; i++)
    {
        const uint32_t r = gammatable[usegamma][*palette++];
        const uint32_t g = gammatable[usegamma][*palette++];
        const uint32_t b = gammatable[usegamma][*palette++];
        const uint32_t c = ((r & 0xF8U) << 8) | ((g & 0xFCU) << 3) | (b >> 3);
        palette2[i] = c | (c << 16);
    }
}

// Given an RGB value, find the closest matching palette index.

int I_GetPaletteIndex (int r, int g, int b)
{
    int best = 0;
    int best_diff = INT_MAX;

    for (int i = 0; i < 256; ++i)
    {
        const uint32_t c = palette2[i] & 0xFFFFU;
        const int cr = (int) ((c >> 8) & 0xF8U);
        const int cg = (int) ((c >> 3) & 0xFCU);
        const int cb = (int) ((c << 3) & 0xF8U);
        const int diff = (r - cr) * (r - cr) + (g - cg) * (g - cg) + (b - cb) * (b - cb);

        if (diff < best_diff)
        {
            best = i;
            best_diff = diff;
        }
        if (diff == 0)
        {
            break;
        }
    }

    return best;
}

void I_BeginRead (void)
{
}

void I_EndRead (void)
{
}

void I_SetWindowTitle (char *title)
{
}

void I_GraphicsCheckCommandLine (void)
{
}

void I_SetGrabMouseCallback (grabmouse_callback_t func)
{
}

void I_EnableLoadingDisk (void)
{
}

void I_BindVideoVariables (void)
{
}

void I_DisplayFPSDots (boolean dots_on)
{
}

void I_CheckIsScreensaver (void)
{
}
