/**
 * @file  fakes.h
 * @brief Host stand-ins for the board: an in-memory "SD card" behind the FatFs
 *        file API, a log that counts messages, and a test signal behind
 *        audio_out_peek().
 */
#ifndef FAKES_H
#define FAKES_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ---- files ---- */

/** Remove all files and reset the error injection. */
void fakefs_reset(void);

/** Add a file (the data is copied). */
void fakefs_add(const char *path, const void *data, size_t size);

/** Add a file with the contents of @p host_path; false if it cannot be read. */
bool fakefs_add_host_file(const char *path, const char *host_path);

/** Make f_read() fail once this many more bytes have been read (0 = never). */
void fakefs_fail_reads_after(size_t bytes);

/* ---- log ---- */

void     fakelog_reset(void);
unsigned fakelog_warnings(void);
unsigned fakelog_errors(void);

/* ---- audio output, as seen by audio_out_peek() ---- */

/** Sine of @p frequency_hz and @p amplitude (0..1 of full scale) on both channels. */
void fakeaudio_sine(float frequency_hz, float amplitude, uint32_t sample_rate);

/** Output stopped: audio_out_peek() returns false. */
void fakeaudio_stop(void);

#endif /* FAKES_H */
