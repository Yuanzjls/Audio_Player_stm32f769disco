/**
 * @file  check.h
 * @brief Minimal test helpers: CHECK macros that report and count failures,
 *        and RUN() to run one test function. A test program returns non-zero
 *        (and CTest reports it failed) if any check failed.
 */
#ifndef CHECK_H
#define CHECK_H

#include <math.h>
#include <stdio.h>
#include <string.h>

extern int g_check_failures;

#define CHECK(cond)                                                                    \
  do {                                                                                 \
    if (!(cond)) {                                                                     \
      fprintf(stderr, "%s:%d: CHECK(%s) failed\n", __FILE__, __LINE__, #cond);         \
      g_check_failures++;                                                              \
    }                                                                                  \
  } while (0)

#define CHECK_EQ_INT(actual, expected)                                                 \
  do {                                                                                 \
    const long long a_ = (long long)(actual), e_ = (long long)(expected);             \
    if (a_ != e_) {                                                                    \
      fprintf(stderr, "%s:%d: %s is %lld, expected %lld\n", __FILE__, __LINE__,        \
              #actual, a_, e_);                                                        \
      g_check_failures++;                                                              \
    }                                                                                  \
  } while (0)

/* |actual - expected| <= tolerance */
#define CHECK_NEAR(actual, expected, tolerance)                                        \
  do {                                                                                 \
    const double a_ = (double)(actual), e_ = (double)(expected);                       \
    if (fabs(a_ - e_) > (double)(tolerance)) {                                         \
      fprintf(stderr, "%s:%d: %s is %g, expected %g +- %g\n", __FILE__, __LINE__,      \
              #actual, a_, e_, (double)(tolerance));                                   \
      g_check_failures++;                                                              \
    }                                                                                  \
  } while (0)

#define CHECK_EQ_STR(actual, expected)                                                 \
  do {                                                                                 \
    const char *a_ = (actual), *e_ = (expected);                                      \
    if (strcmp(a_, e_) != 0) {                                                         \
      fprintf(stderr, "%s:%d: %s is \"%s\", expected \"%s\"\n", __FILE__, __LINE__,    \
              #actual, a_, e_);                                                        \
      g_check_failures++;                                                              \
    }                                                                                  \
  } while (0)

#define RUN(test)                                                                      \
  do {                                                                                 \
    const int before_ = g_check_failures;                                              \
    test();                                                                            \
    printf("%s %s\n", (g_check_failures == before_) ? "  ok  " : "  FAIL", #test);     \
  } while (0)

/* In each test program: int main(void) { RUN(...); ...; return CHECK_RESULT(); } */
#define CHECK_RESULT() (g_check_failures == 0 ? 0 : 1)

#endif /* CHECK_H */
