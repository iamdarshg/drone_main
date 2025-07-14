// watchdog_m0.c - M0 core: hardware watchdog and monitoring
#include "watchdog_m0.h"
#include <stdint.h>

volatile uint32_t m0_watchdog_counter = 0;
volatile uint32_t m0_status_flags = 0;

void m0_watchdog_kick(void) {
    m0_watchdog_counter = 0;
}

void m0_watchdog_task(void) {
    // This function should be called in a loop on the M0 core
    m0_watchdog_counter++;
    if (m0_watchdog_counter > 1000000) {
        m0_status_flags |= 0x01; // Set timeout flag
        // Optionally trigger system reset or alert M4
    }
    // Add more monitoring (e.g., bus, RAM, error flags) as needed
}

uint32_t m0_get_status(void) {
    return m0_status_flags;
}
