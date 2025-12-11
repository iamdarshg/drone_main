#ifndef SELFTEST_M0_H
#define SELFTEST_M0_H
#include <stdint.h>
void m0_selftest_run(void);
void m0_error_set(uint32_t err);
void m0_error_clear(uint32_t err);
uint32_t m0_error_get(void);
#endif
