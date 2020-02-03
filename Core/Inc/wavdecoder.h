#ifndef _wavdecoder_h_
#define _wavdecoder_h_
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct 
{
    /* data */
    uint8_t ChunkID[4];
    uint32_t ChunkSize;
    uint8_t Format[4];
    uint8_t Subchunk1ID[4];
    uint32_t Subchunk1Size;
    uint16_t AudioFormat;
    uint16_t NumChannels;
    uint32_t Samplerate;
    uint32_t ByteRate;
    uint16_t BlockAlign;
    uint16_t BitsPersample;
    uint8_t Subchunk2ID[4];
    uint32_t Subchunk2Size;
}Wavheader;

#ifdef __cplusplus
}
#endif
#endif
