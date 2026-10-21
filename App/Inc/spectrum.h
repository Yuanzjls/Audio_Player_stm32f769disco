/**
 * @file  spectrum.h
 * @brief Live spectrum of the audio being played (for the UI).
 */
#ifndef SPECTRUM_H
#define SPECTRUM_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SPECTRUM_BANDS 32

typedef struct
{
  float level[SPECTRUM_BANDS];   /* 0..1, smoothed bar heights        */
  float peak[SPECTRUM_BANDS];    /* 0..1, slowly falling peak markers */
} spectrum_t;

void spectrum_init(void);

/**
 * Analyse the samples the audio output is about to play and update @p s.
 * @p sample_rate maps FFT bins to frequencies, @p now_ms drives the
 * smoothing. When nothing is playing the bars fall to zero.
 * Returns true if @p s changed (the display needs redrawing).
 */
bool spectrum_update(spectrum_t *s, uint32_t sample_rate, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* SPECTRUM_H */
