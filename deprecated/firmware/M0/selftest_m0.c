// selftest_m0.c - M0 core: hardware self-test and error handling
#include "selftest_m0.h"
#include <stdint.h>

volatile uint32_t m0_error_flags = 0;

void m0_selftest_run(void) {
    // Example: check a hardware register or memory region
    // Set error flag if test fails
    if (/* hardware test fails */ 0) {
        m0_error_flags |= 0x01;
    }
    // Add more tests as needed
}

void m0_error_set(uint32_t err) {
    m0_error_flags |= err;
}

void m0_error_clear(uint32_t err) {
    m0_error_flags &= ~err;
}

uint32_t m0_error_get(void) {
    return m0_error_flags;
}
