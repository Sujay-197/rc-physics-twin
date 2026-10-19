/* Minimal test macros: no framework, so the C test stays plain C. */
#ifndef RC_TWIN_TEST_CHECK_H
#define RC_TWIN_TEST_CHECK_H

#include <stdio.h>

static int g_failures = 0;
static int g_checks = 0;

#define CHECK(cond)                                                     \
  do {                                                                  \
    ++g_checks;                                                         \
    if (!(cond)) {                                                      \
      ++g_failures;                                                     \
      fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__,  \
              #cond);                                                   \
    }                                                                   \
  } while (0)

#define TEST_MAIN_END()                                                 \
  do {                                                                  \
    printf("%d checks, %d failed\n", g_checks, g_failures);             \
    return g_failures == 0 ? 0 : 1;                                     \
  } while (0)

#endif
