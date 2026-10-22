/**
 * @file  inflate.h
 * @brief Streaming DEFLATE decompression (RFC 1951), as in PNG and zlib data.
 *
 * The compressed data is pulled through read() in pieces of any size, and the
 * output is pushed through write() in pieces of up to 32 KB, so neither the
 * input nor the output has to fit in memory: only the 32 KB window does.
 */
#ifndef INFLATE_H
#define INFLATE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define INFLATE_WINDOW 32768U

/** Next piece of the compressed data in @p *data; returns its length, 0 at the end */
typedef uint32_t (*inflate_read_t)(void *ctx, const uint8_t **data);

/** Decompressed bytes; false stops the decompression */
typedef bool (*inflate_write_t)(void *ctx, const uint8_t *data, uint32_t len);

typedef enum
{
  INFLATE_OK,          /**< the last block was decoded              */
  INFLATE_STOPPED,     /**< write() returned false                  */
  INFLATE_TRUNCATED,   /**< read() ran out before the end           */
  INFLATE_ERROR,       /**< not valid DEFLATE data                  */
} inflate_result_t;

typedef struct
{
  uint16_t count[16];          /* codes of each length */
  uint16_t symbol[288];        /* symbols in code order */
  uint16_t fast[1U << 9];      /* low 9 bits of the input -> symbol | length << 9; 0: longer code */
} inflate_huffman_t;

/** State: about 36 KB, keep it out of task stacks */
typedef struct
{
  inflate_read_t  read;
  inflate_write_t write;
  void           *ctx;

  const uint8_t  *in, *in_end;
  uint32_t        bits;        /* bit buffer, next bit in bit 0 */
  uint32_t        nbits;
  uint32_t        pad_bits;    /* zero bits added past the end of the input */
  bool            failed;

  uint32_t        wpos;        /* write position in the window */
  uint32_t        total;       /* bytes produced, up to the window size */
  bool            stopped;

  inflate_huffman_t lit, dist;
  uint8_t         window[INFLATE_WINDOW];
} inflate_t;

/** Decompress one raw DEFLATE stream (no zlib header). */
inflate_result_t inflate_run(inflate_t *s, inflate_read_t read, inflate_write_t write, void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* INFLATE_H */
