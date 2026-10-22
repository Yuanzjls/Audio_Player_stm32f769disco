/**
 * @file  simd8.h
 * @brief Four bytes at a time in a 32-bit word: single SIMD instructions on
 *        the Cortex-M7 (UADD8, UHADD8, USAD8), plain C with the same results
 *        elsewhere (host tests).
 */
#ifndef SIMD8_H
#define SIMD8_H

#include <stdint.h>
#include <string.h>

/* Unaligned 32-bit loads and stores (single LDR / STR on the Cortex-M7) */
static inline uint32_t load32(const uint8_t *p)    { uint32_t v; memcpy(&v, p, 4); return v; }
static inline void store32(uint8_t *p, uint32_t v) { memcpy(p, &v, 4); }

#if defined(__ARM_FEATURE_SIMD32)
#include <arm_acle.h>
/** Byte-wise a + b, modulo 256 */
static inline uint32_t add8(uint32_t a, uint32_t b)  { return (uint32_t)__uadd8((uint8x4_t)a, (uint8x4_t)b); }
/** Byte-wise (a + b) / 2, rounded down (no overflow) */
static inline uint32_t havg8(uint32_t a, uint32_t b) { return (uint32_t)__uhadd8((uint8x4_t)a, (uint8x4_t)b); }
/** Sum of the four bytes */
static inline uint32_t sum8x4(uint32_t a)            { return __usad8((uint8x4_t)a, (uint8x4_t)0U); }
#else
static inline uint32_t add8(uint32_t a, uint32_t b)
{
  return ((a & 0x7F7F7F7FU) + (b & 0x7F7F7F7FU)) ^ ((a ^ b) & 0x80808080U);
}
static inline uint32_t havg8(uint32_t a, uint32_t b)
{
  return (a & b) + (((a ^ b) & 0xFEFEFEFEU) >> 1);
}
static inline uint32_t sum8x4(uint32_t a)
{
  return (a & 0xFFU) + ((a >> 8) & 0xFFU) + ((a >> 16) & 0xFFU) + (a >> 24);
}
#endif

#endif /* SIMD8_H */
