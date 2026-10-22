/**
 * @file  audio_out.h
 * @brief Headphone output: SAI1 block A (I2S-like, 16-bit stereo) feeding the
 *        WM8994 codec, with a circular DMA double buffer.
 *
 * The caller owns a buffer split in two halves. While the DMA plays one half,
 * the callback reports which half has just been played so it can be refilled.
 */
#ifndef AUDIO_OUT_H
#define AUDIO_OUT_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  AUDIO_OUT_HALF_DONE,  /**< First half played: refill it  */
  AUDIO_OUT_FULL_DONE,  /**< Second half played: refill it */
} audio_out_event_t;

/** Called from the DMA interrupt. Keep it short (e.g. notify a task). */
typedef void (*audio_out_callback_t)(audio_out_event_t event);

/** True if the sample rate can be generated exactly by the clock tree and codec. */
bool audio_out_is_rate_supported(uint32_t sample_rate);

/**
 * (Re)configure clocks, SAI and codec for @p sample_rate. Any running
 * playback is stopped first. @p volume is 0..100.
 */
bool audio_out_configure(uint32_t sample_rate, uint8_t volume, audio_out_callback_t callback);

/**
 * Start circular playback of @p buffer (interleaved L/R int16 samples).
 * @p size_bytes is the size of the whole buffer (both halves).
 */
bool audio_out_start(int16_t *buffer, uint32_t size_bytes);

bool audio_out_pause(void);
bool audio_out_resume(void);
void audio_out_stop(void);

/** @p volume is 0..100 */
bool audio_out_set_volume(uint8_t volume);

/**
 * Copy the next @p frames stereo frames the DMA is about to play (starting at
 * its current position), e.g. for a spectrum display. Returns false when no
 * audio is playing (stopped or paused).
 */
bool audio_out_peek(int16_t *out, uint32_t frames);

/** Stereo frames of the buffer played since the DMA last wrapped around it. */
uint32_t audio_out_position(void);

/** Output errors since audio_out_start() */
typedef struct
{
  uint32_t underruns;    /**< checks that found the SAI FIFO had run empty (at most one per check) */
  uint32_t dma_errors;   /**< DMA transfer / FIFO errors */
} audio_out_errors_t;

/**
 * Check the SAI for an underrun since the last call (it garbles the sound)
 * and copy the error counts. A task calls it regularly while playing, e.g.
 * after each refill: underruns are polled, the SAI interrupt is not used.
 */
void audio_out_errors(audio_out_errors_t *errors);

/** To be called from DMA2_Stream6_IRQHandler */
void audio_out_dma_irq_handler(void);

#ifdef __cplusplus
}
#endif

#endif /* AUDIO_OUT_H */
