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
//    Sound on the STM32F769 Discovery's headphone output, at 11025 Hz like
//    the originals. A task of its own fills each half of the audio DMA
//    buffer as soon as it has been played (the DMA interrupt only wakes
//    it): the music, rendered by the OPL emulator from the game's MUS
//    songs (opl_stm32.c, i_oplmusic.c), plus the sound effects (8-bit
//    samples in the WAD), with volume and stereo separation per channel.
//

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "config.h"
#include "deh_str.h"
#include "doomtype.h"
#include "i_sound.h"
#include "m_misc.h"
#include "w_wad.h"
#include "z_zone.h"
#include "doom_port.h"   // after the engine's headers: they define false and true
#include "audio_out.h"
#include "main.h"

#define OUT_RATE     11025
#define HALF_FRAMES  256              // 23 ms per DMA buffer half
#define NUM_CHANNELS 16
#define TASK_STACK_WORDS 1024
#define TASK_PRIORITY    3            // the music player's: a half must be ready in 23 ms

#define NOTIFY_HALF  (1UL << 0)
#define NOTIFY_FULL  (1UL << 1)

// The music, as the OPL music player uses it
extern music_module_t music_opl_module;
void OPL_STM32_Render(int32_t *out, unsigned int nsamples);

int snd_sfxdevice = SNDDEVICE_SB;
int snd_musicdevice = SNDDEVICE_SB;   // OPL music (and the OPL version of the intro tune)
int snd_samplerate = OUT_RATE;
int snd_cachesize = 0;
int snd_maxslicetime_ms = 28;
char *snd_musiccmd = "";
int snd_pitchshift = 0;

// A playing effect. The sound task reads it while active is set; the engine
// changes it only while active is clear (or just the volumes, word by word).

typedef struct
{
    volatile boolean active;
    const uint8_t   *data;          // unsigned 8-bit samples
    uint32_t         len;           // samples
    uint32_t         pos;           // 16.16 fixed point
    uint32_t         step;          // per output sample, 16.16
    volatile int32_t vol_l, vol_r;  // 0..254
    int              lumpnum;       // cached while playing, -1: none
} channel_t;

static channel_t    channels[NUM_CHANNELS];
static int16_t      dma_buf[2 * HALF_FRAMES * 2] DMA_BUFFER;
static int32_t      music_buf[HALF_FRAMES];   // mono, from the OPL emulator
static boolean      sfx_prefix;
static boolean      initialized;
static boolean      music_ready;              // the OPL music player is set up
static TaskHandle_t sound_task;

// ---- mixing (sound task) ----------------------------------------------------

static inline int16_t Clamp16(int32_t v)
{
    return (int16_t) (v > 32767 ? 32767 : (v < -32768 ? -32768 : v));
}

static void Mix(int16_t *out)
{
    channel_t *playing[NUM_CHANNELS];
    int n = 0;

    for (int c = 0; c < NUM_CHANNELS; c++)
    {
        if (channels[c].active)
        {
            playing[n++] = &channels[c];
        }
    }

    // The music first (it also runs the music player's timed events)
    if (music_ready)
    {
        OPL_STM32_Render(music_buf, HALF_FRAMES);
    }
    else
    {
        memset(music_buf, 0, sizeof(music_buf));
    }

    for (int i = 0; i < HALF_FRAMES; i++)
    {
        int32_t l = music_buf[i], r = music_buf[i];
        for (int k = 0; k < n; k++)
        {
            channel_t *ch = playing[k];
            if (ch == NULL)
            {
                continue;
            }
            const uint32_t s = ch->pos >> 16;
            if (s >= ch->len)
            {
                ch->active = false;     // the engine releases the lump
                playing[k] = NULL;
                continue;
            }
            const int32_t v = (int32_t) ch->data[s] - 128;
            l += v * ch->vol_l;
            r += v * ch->vol_r;
            ch->pos += ch->step;
        }
        out[i * 2]     = Clamp16(l);
        out[i * 2 + 1] = Clamp16(r);
    }
}

// DMA interrupt: a half was played; the task refills it
static void AudioCallback(audio_out_event_t event)
{
    if (sound_task != NULL)
    {
        BaseType_t woken = pdFALSE;
        xTaskNotifyFromISR(sound_task, (event == AUDIO_OUT_HALF_DONE) ? NOTIFY_HALF : NOTIFY_FULL,
                           eSetBits, &woken);
        portYIELD_FROM_ISR(woken);
    }
}

static void SoundTask(void *arg)
{
    for (;;)
    {
        uint32_t halves = 0;
        xTaskNotifyWait(0, UINT32_MAX, &halves, portMAX_DELAY);
        if (halves & NOTIFY_HALF)
        {
            Mix(&dma_buf[0]);
        }
        if (halves & NOTIFY_FULL)
        {
            Mix(&dma_buf[HALF_FRAMES * 2]);
        }
    }
}

// The app turns the output on when the game runs and off when it pauses

void I_SoundOutput(bool on, uint8_t volume)
{
    if (!on)
    {
        audio_out_stop();
        return;
    }
    memset(dma_buf, 0, sizeof(dma_buf));
    if (audio_out_configure(OUT_RATE, volume, AudioCallback))
    {
        audio_out_start(dma_buf, sizeof(dma_buf));
    }
}

// ---- channels (engine) ------------------------------------------------------

// Give back the lumps of the effects that have finished
static void ReleaseFinished(void)
{
    for (int c = 0; c < NUM_CHANNELS; c++)
    {
        if (!channels[c].active && channels[c].lumpnum >= 0)
        {
            W_ReleaseLumpNum(channels[c].lumpnum);
            channels[c].lumpnum = -1;
        }
    }
}

static void SetVolume(channel_t *ch, int vol, int sep)
{
    if (sep < 0) sep = 0;
    if (sep > 254) sep = 254;
    if (vol < 0) vol = 0;
    if (vol > 127) vol = 127;

    ch->vol_l = ((254 - sep) * vol) / 127;
    ch->vol_r = (sep * vol) / 127;
}

void I_InitSound(boolean use_sfx_prefix)
{
    sfx_prefix = use_sfx_prefix;
    for (int c = 0; c < NUM_CHANNELS; c++)
    {
        channels[c].lumpnum = -1;
    }
    initialized = true;

    if (sound_task == NULL &&
        xTaskCreate(SoundTask, "dsound", TASK_STACK_WORDS, NULL, TASK_PRIORITY, &sound_task) != pdPASS)
    {
        fprintf(stderr, "I_InitSound: no memory for the sound task\n");
    }
}

void I_ShutdownSound(void)
{
    for (int c = 0; c < NUM_CHANNELS; c++)
    {
        channels[c].active = false;
    }
    ReleaseFinished();
}

int I_GetSfxLumpNum(sfxinfo_t *sfxinfo)
{
    char namebuf[9];

    if (sfxinfo->link != NULL)
    {
        sfxinfo = sfxinfo->link;
    }

    if (sfx_prefix)
    {
        M_snprintf(namebuf, sizeof(namebuf), "ds%s", DEH_String(sfxinfo->name));
    }
    else
    {
        M_StringCopy(namebuf, DEH_String(sfxinfo->name), sizeof(namebuf));
    }

    return W_GetNumForName(namebuf);
}

void I_UpdateSound(void)
{
    ReleaseFinished();
}

void I_UpdateSoundParams(int channel, int vol, int sep)
{
    if (initialized && channel >= 0 && channel < NUM_CHANNELS)
    {
        SetVolume(&channels[channel], vol, sep);
    }
}

int I_StartSound(sfxinfo_t *sfxinfo, int channel, int vol, int sep, int pitch)
{
    if (!initialized || channel < 0 || channel >= NUM_CHANNELS)
    {
        return -1;
    }

    channel_t *ch = &channels[channel];
    ch->active = false;
    ReleaseFinished();

    // DMX format: 3, sample rate, length, then the samples; the DMX library
    // skips 16 bytes at each end
    const int lumpnum = sfxinfo->lumpnum;
    const byte *lump = W_CacheLumpNum(lumpnum, PU_STATIC);
    const unsigned int lumplen = W_LumpLength(lumpnum);
    if (lumplen < 8 || lump[0] != 0x03 || lump[1] != 0x00)
    {
        W_ReleaseLumpNum(lumpnum);
        return -1;
    }
    const uint32_t rate = (uint32_t) (lump[3] << 8 | lump[2]);
    const uint32_t length = (uint32_t) lump[7] << 24 | (uint32_t) lump[6] << 16 | (uint32_t) lump[5] << 8 | lump[4];
    if (length > lumplen - 8 || length <= 48 || rate == 0)
    {
        W_ReleaseLumpNum(lumpnum);
        return -1;
    }

    ch->data    = lump + 8 + 16;
    ch->len     = length - 32;
    ch->pos     = 0;
    ch->step    = (rate << 16) / OUT_RATE;
    ch->lumpnum = lumpnum;
    SetVolume(ch, vol, sep);
    __DMB();
    ch->active = true;
    return channel;
}

void I_StopSound(int channel)
{
    if (initialized && channel >= 0 && channel < NUM_CHANNELS)
    {
        channels[channel].active = false;
        ReleaseFinished();
    }
}

boolean I_SoundIsPlaying(int channel)
{
    return initialized && channel >= 0 && channel < NUM_CHANNELS && channels[channel].active;
}

void I_PrecacheSounds(sfxinfo_t *sounds, int num_sounds)
{
}

void I_BindSoundVariables(void)
{
}

// ---- music: the OPL music player --------------------------------------------

void I_InitMusic(void)
{
    // Needs the sound task (its timer detection waits for rendered audio)
    music_ready = sound_task != NULL && music_opl_module.Init();
    if (!music_ready)
    {
        fprintf(stderr, "I_InitMusic: no OPL music\n");
    }
}

void I_ShutdownMusic(void)
{
    if (music_ready)
    {
        music_ready = false;
        music_opl_module.Shutdown();
    }
}

void I_SetMusicVolume(int volume)
{
    if (music_ready)
    {
        music_opl_module.SetMusicVolume(volume);
    }
}

void I_PauseSong(void)
{
    if (music_ready)
    {
        music_opl_module.PauseMusic();
    }
}

void I_ResumeSong(void)
{
    if (music_ready)
    {
        music_opl_module.ResumeMusic();
    }
}

void *I_RegisterSong(void *data, int len)
{
    return music_ready ? music_opl_module.RegisterSong(data, len) : NULL;
}

void I_UnRegisterSong(void *handle)
{
    if (music_ready)
    {
        music_opl_module.UnRegisterSong(handle);
    }
}

void I_PlaySong(void *handle, boolean looping)
{
    if (music_ready)
    {
        music_opl_module.PlaySong(handle, looping);
    }
}

void I_StopSong(void)
{
    if (music_ready)
    {
        music_opl_module.StopSong();
    }
}

boolean I_MusicIsPlaying(void)
{
    return music_ready && music_opl_module.MusicIsPlaying();
}
