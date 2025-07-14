// watchdog_m0.c - M0 core: hardware watchdog and monitoring
#include "watchdog_m0.h"
#include <stdint.h>

volatile uint32_t m0_watchdog_counter = 0;
volatile uint32_t m0_status_flags = 0;

// Add: Monitor task priorities and PID loop timing
volatile uint32_t m0_pid_delay_flag = 0;
volatile uint32_t m0_task_priority_flags = 0;

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

void m0_monitor_pid_loop(uint32_t last_pid_tick, uint32_t now, uint32_t max_delay) {
    if ((now - last_pid_tick) > max_delay) {
        m0_pid_delay_flag = 1;
        // Optionally trigger alert or raise priority
    } else {
        m0_pid_delay_flag = 0;
    }
}

void m0_set_task_priority_flag(uint32_t flag) {
    m0_task_priority_flags |= flag;
}
void m0_clear_task_priority_flag(uint32_t flag) {
    m0_task_priority_flags &= ~flag;
}
uint32_t m0_get_task_priority_flags(void) {
    return m0_task_priority_flags;
}
uint32_t m0_get_pid_delay_flag(void) {
    return m0_pid_delay_flag;
}

uint32_t m0_get_status(void) {
    return m0_status_flags;
}
