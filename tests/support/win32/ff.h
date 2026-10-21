/**
 * @file  ff.h (tests, Windows hosts only)
 * @brief FatFs's ff.h takes a Visual C++ branch when _WIN32 is defined
 *        (windows.h types, macro clashes). Hide _WIN32 while it is read so
 *        that every host uses its standard C99 branch, as the firmware does.
 */
#ifndef TESTS_FF_WRAPPER_H
#define TESTS_FF_WRAPPER_H

#ifdef _WIN32
#undef _WIN32
#include "../../../FATFS/ff.h"
#define _WIN32 1
#else
#include "../../../FATFS/ff.h"
#endif

#endif /* TESTS_FF_WRAPPER_H */
