/**
 * @file  test_spectrum.c
 * @brief Spectrum analyser: a sine lights the band of its frequency at the
 *        right level; bars rise at once and fall at a steady rate; peak
 *        markers hold, then sink. The audio comes from fakeaudio_sine().
 */
#include <math.h>
#include <stdint.h>

#include "check.h"
#include "fakes.h"
#include "spectrum.h"

#define FRAME_MS 33U

static spectrum_t s_spec;
static uint32_t   s_now_ms = 1000;

/* Band the analyser puts @p hz in (log-spaced from 40 Hz to 16 kHz or 0.95 x Nyquist) */
static int expected_band(double hz, uint32_t rate)
{
  const double high = fmin(16000.0, 0.95 * rate / 2.0);
  return (int)floor(SPECTRUM_BANDS * log(hz / 40.0) / log(high / 40.0));
}

static bool step(uint32_t sample_rate)
{
  s_now_ms += FRAME_MS;
  return spectrum_update(&s_spec, sample_rate, s_now_ms);
}

static void run_for_ms(uint32_t ms, uint32_t sample_rate)
{
  for (uint32_t t = 0; t < ms; t += FRAME_MS)
  {
    step(sample_rate);
  }
}

static int loudest_band(void)
{
  int best = 0;
  for (int b = 1; b < SPECTRUM_BANDS; b++)
  {
    if (s_spec.level[b] > s_spec.level[best])
    {
      best = b;
    }
  }
  return best;
}

/* Silence long enough for every bar and peak to reach the bottom */
static void settle(void)
{
  fakeaudio_stop();
  run_for_ms(3000, 44100);
}

static void test_full_scale_tone(void)
{
  settle();
  fakeaudio_sine(1000.0f, 1.0f, 44100);
  run_for_ms(200, 44100);

  const int band = expected_band(1000.0, 44100);
  CHECK_EQ_INT(loudest_band(), band);
  CHECK(s_spec.level[band] > 0.9f);             /* about 0 dBFS */
  CHECK(s_spec.level[2] < 0.05f);               /* far below: nothing */
  CHECK(s_spec.level[SPECTRUM_BANDS - 2] < 0.05f);
}

/* Levels are on a 60 dB scale: -40 dBFS is a third of the height */
static void test_level_scale(void)
{
  settle();
  fakeaudio_sine(1000.0f, 0.01f, 44100);
  run_for_ms(200, 44100);
  CHECK_NEAR(s_spec.level[expected_band(1000.0, 44100)], (60.0 - 40.0) / 60.0, 0.05);
}

static void test_other_frequencies_and_rates(void)
{
  static const float freqs[] = { 100.0f, 440.0f, 5000.0f, 12000.0f };
  for (size_t i = 0; i < sizeof(freqs) / sizeof(freqs[0]); i++)
  {
    settle();
    fakeaudio_sine(freqs[i], 0.5f, 44100);
    run_for_ms(200, 44100);
    const int band = loudest_band();
    if (band != expected_band(freqs[i], 44100))
    {
      fprintf(stderr, "    %.0f Hz:\n", (double)freqs[i]);
      CHECK_EQ_INT(band, expected_band(freqs[i], 44100));
    }
  }

  /* 48 kHz: the bands are recomputed for the new bin spacing */
  settle();
  fakeaudio_sine(1000.0f, 0.5f, 48000);
  run_for_ms(200, 48000);
  CHECK_EQ_INT(loudest_band(), expected_band(1000.0, 48000));

  /* 16 kHz audio: the top band ends below Nyquist */
  settle();
  fakeaudio_sine(7000.0f, 0.5f, 16000);
  run_for_ms(200, 16000);
  CHECK_EQ_INT(loudest_band(), expected_band(7000.0, 16000));
}

/* Bars rise at once, then fall 1.8 per second; peaks hold 400 ms, then sink */
static void test_rise_fall_and_peaks(void)
{
  settle();
  const int band = expected_band(1000.0, 44100);
  fakeaudio_sine(1000.0f, 1.0f, 44100);
  CHECK(step(44100));
  const float top = s_spec.level[band];
  CHECK(top > 0.9f);                             /* one frame: full height */
  CHECK_NEAR(s_spec.peak[band], top, 1e-6);

  fakeaudio_stop();
  step(44100);
  CHECK_NEAR(s_spec.level[band], top - 1.8 * FRAME_MS / 1000.0, 0.01);

  run_for_ms(250, 44100);                        /* ~300 ms after the top */
  CHECK_NEAR(s_spec.peak[band], top, 1e-6);      /* the peak still holds */
  run_for_ms(400, 44100);
  CHECK(s_spec.peak[band] < top - 0.05f);        /* now sinking */
  CHECK(s_spec.peak[band] >= s_spec.level[band]);

  run_for_ms(3000, 44100);
  CHECK_NEAR(s_spec.level[band], 0.0, 1e-6);
  CHECK_NEAR(s_spec.peak[band], 0.0, 1e-6);
}

/* Nothing playing and everything at rest: no change, so no redraw */
static void test_idle_reports_no_change(void)
{
  settle();
  CHECK(!step(44100));
  CHECK(!step(0));                                /* no track at all */
}

int main(void)
{
  spectrum_init();
  RUN(test_full_scale_tone);
  RUN(test_level_scale);
  RUN(test_other_frequencies_and_rates);
  RUN(test_rise_fall_and_peaks);
  RUN(test_idle_reports_no_change);
  return CHECK_RESULT();
}
