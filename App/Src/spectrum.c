/**
 * @file  spectrum.c
 * @brief 32-band spectrum analyser with CMSIS-DSP.
 *
 * Each update takes the next 1024 samples the DMA is about to play (so the
 * bars match what is heard), mixes them to mono, applies a Hann window and a
 * real FFT, and groups the bins into logarithmically spaced bands from 40 Hz
 * to 16 kHz (or Nyquist). Band levels are shown on a 60 dB scale; bars rise
 * instantly and fall at a fixed rate, peak markers hold briefly then sink.
 */
#include "spectrum.h"

#include <math.h>
#include <string.h>

#include "arm_math.h"
#include "audio_out.h"

#define FFT_SIZE          1024U
#define BINS              (FFT_SIZE / 2U)

#define FREQ_LOW_HZ       40.0f
#define FREQ_HIGH_HZ      16000.0f
#define RANGE_DB          60.0f      /* 0 dBFS at the top, -60 dBFS at the bottom */

#define FALL_PER_S        1.8f       /* bar height units (0..1) per second */
#define PEAK_HOLD_MS      400U
#define PEAK_FALL_PER_S   0.6f
#define CHANGE_EPSILON    0.002f

static arm_rfft_fast_instance_f32 s_fft;
static float    s_window[FFT_SIZE];
static float    s_time[FFT_SIZE];      /* windowed input, then reused for magnitudes */
static float    s_freq[FFT_SIZE];      /* packed complex FFT output                  */
static int16_t  s_pcm[2U * FFT_SIZE];  /* stereo samples from the output buffer      */

static uint16_t s_band_lo[SPECTRUM_BANDS];
static uint16_t s_band_hi[SPECTRUM_BANDS];
static uint32_t s_band_rate;
static uint32_t s_peak_hold[SPECTRUM_BANDS];
static uint32_t s_last_ms;

void spectrum_init(void)
{
  arm_rfft_fast_init_1024_f32(&s_fft);
  for (uint32_t i = 0; i < FFT_SIZE; i++)
  {
    s_window[i] = 0.5f * (1.0f - cosf(2.0f * PI * (float)i / (float)(FFT_SIZE - 1U)));
  }
}

/* Bin ranges of the log-spaced bands for this sample rate */
static void compute_bands(uint32_t sample_rate)
{
  const float nyquist = 0.5f * (float)sample_rate;
  const float high    = (FREQ_HIGH_HZ < 0.95f * nyquist) ? FREQ_HIGH_HZ : 0.95f * nyquist;
  const float hz_per_bin = (float)sample_rate / (float)FFT_SIZE;

  for (uint32_t b = 0; b < SPECTRUM_BANDS; b++)
  {
    const float f0 = FREQ_LOW_HZ * powf(high / FREQ_LOW_HZ, (float)b / SPECTRUM_BANDS);
    const float f1 = FREQ_LOW_HZ * powf(high / FREQ_LOW_HZ, (float)(b + 1U) / SPECTRUM_BANDS);
    uint32_t lo = (uint32_t)(f0 / hz_per_bin);
    uint32_t hi = (uint32_t)ceilf(f1 / hz_per_bin);
    if (lo < 1U)
    {
      lo = 1U;               /* skip DC */
    }
    if (hi <= lo)
    {
      hi = lo + 1U;          /* low bands are narrower than one bin */
    }
    if (hi > BINS)
    {
      hi = BINS;
    }
    s_band_lo[b] = (uint16_t)lo;
    s_band_hi[b] = (uint16_t)hi;
  }
  s_band_rate = sample_rate;
}

/* Target band heights (0..1) of the audio about to play; false if none is playing */
static bool analyse(float *target, uint32_t sample_rate)
{
  if (sample_rate == 0U || !audio_out_peek(s_pcm, FFT_SIZE))
  {
    return false;
  }
  if (sample_rate != s_band_rate)
  {
    compute_bands(sample_rate);
  }

  for (uint32_t i = 0; i < FFT_SIZE; i++)
  {
    const float mono = ((float)s_pcm[2U * i] + (float)s_pcm[2U * i + 1U]) * (0.5f / 32768.0f);
    s_time[i] = mono * s_window[i];
  }
  arm_rfft_fast_f32(&s_fft, s_time, s_freq, 0);

  /* s_freq = [DC, Nyquist, re1, im1, re2, im2, ...]: squared magnitudes of bins
   * 1..BINS-1 into s_time[1..] (no square roots: only the loudest bin of a band
   * is converted to dB) */
  float *pow2 = s_time;
  arm_cmplx_mag_squared_f32(&s_freq[2], &pow2[1], BINS - 1U);

  /* A full-scale sine gives a magnitude of FFT_SIZE/4 with the Hann window */
  const float scale2 = (4.0f / (float)FFT_SIZE) * (4.0f / (float)FFT_SIZE);
  for (uint32_t b = 0; b < SPECTRUM_BANDS; b++)
  {
    float v = 0.0f;
    for (uint32_t k = s_band_lo[b]; k < s_band_hi[b] && k < BINS; k++)
    {
      if (pow2[k] > v)
      {
        v = pow2[k];
      }
    }
    const float db = 10.0f * log10f(v * scale2 + 1e-18f);
    float level = (db + RANGE_DB) / RANGE_DB;
    target[b] = (level < 0.0f) ? 0.0f : (level > 1.0f ? 1.0f : level);
  }
  return true;
}

bool spectrum_update(spectrum_t *s, uint32_t sample_rate, uint32_t now_ms)
{
  float target[SPECTRUM_BANDS];
  const uint32_t dt_ms = (s_last_ms != 0U) ? now_ms - s_last_ms : 0U;
  const float dt = (float)dt_ms / 1000.0f;
  s_last_ms = now_ms;

  if (!analyse(target, sample_rate))
  {
    memset(target, 0, sizeof(target));
  }

  bool changed = false;
  for (uint32_t b = 0; b < SPECTRUM_BANDS; b++)
  {
    const float old_level = s->level[b];
    const float old_peak  = s->peak[b];

    /* Bars: instant attack, constant fall */
    float level = old_level - FALL_PER_S * dt;
    if (target[b] > level)
    {
      level = target[b];
    }
    s->level[b] = (level > 0.0f) ? level : 0.0f;

    /* Peaks: follow the bar up, hold, then sink */
    if (s->level[b] >= s->peak[b])
    {
      s->peak[b]      = s->level[b];
      s_peak_hold[b]  = PEAK_HOLD_MS;
    }
    else if (s_peak_hold[b] > dt_ms)
    {
      s_peak_hold[b] -= dt_ms;
    }
    else
    {
      s_peak_hold[b] = 0;
      const float p = s->peak[b] - PEAK_FALL_PER_S * dt;
      s->peak[b] = (p > s->level[b]) ? p : s->level[b];
    }

    if (fabsf(s->level[b] - old_level) > CHANGE_EPSILON || fabsf(s->peak[b] - old_peak) > CHANGE_EPSILON)
    {
      changed = true;
    }
  }
  return changed;
}
