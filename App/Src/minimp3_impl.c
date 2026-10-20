/**
 * @file  minimp3_impl.c
 * @brief The minimp3 implementation (single-header library, compiled once).
 *
 * MINIMP3_ONLY_MP3 and MINIMP3_NO_SIMD are set for the whole project in
 * CMakeLists.txt (Layer III only; the Cortex-M7 has no NEON).
 */
#define MINIMP3_IMPLEMENTATION
#include "minimp3.h"
