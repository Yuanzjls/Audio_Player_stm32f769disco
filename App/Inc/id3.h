/**
 * @file  id3.h
 * @brief Title / artist from ID3v2 (2.2, 2.3, 2.4) and ID3v1 tags.
 */
#ifndef ID3_H
#define ID3_H

#include <stdbool.h>
#include <stdint.h>

#include "ff.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ID3_TEXT_MAX 128

typedef struct
{
  char title[ID3_TEXT_MAX];    /* UTF-8, empty if absent */
  char artist[ID3_TEXT_MAX];
} id3_tags_t;

/**
 * Parse an ID3v2 tag at the start of @p file. Fills the fields it finds.
 * Returns the total tag size in bytes (where the audio starts), 0 if none.
 */
uint32_t id3v2_parse(FIL *file, id3_tags_t *tags);

/**
 * Check for an ID3v1 tag in the last 128 bytes of the file. Fills only the
 * fields that are still empty. Returns true if the tag is present.
 */
bool id3v1_parse(FIL *file, id3_tags_t *tags);

#ifdef __cplusplus
}
#endif

#endif /* ID3_H */
