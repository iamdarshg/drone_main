// watchdog_m0.c - M0 core: hardware watchdog and monitoring
#include "watchdog_m0.h"
#include "selftest_m0.h"
#include "bus_monitor.h"
#include "imu_monitor.h"
#include "rf_monitor.h"
#include "ram_monitor.h"
#include "gps_uart.h"
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

void m0_check_all_hardware(void) {
    // Bus contention
    if (bus_monitor_get_spi_bytes() > 1000000 || bus_monitor_get_i2c_bytes() > 1000000) {
        m0_error_set(0x01); // Bus error
        bus_monitor_reset();
    }

    // IMU lockup
    imu_monitor_check_all();
    if (imu_monitor_get_errors()) m0_error_set(0x02);

    // IMU FIFO and read rate checks (KX122, LSM6DS3, LSM303C)
#ifdef IMU_KX122_PRESENT
    if (!imu_kx122_fifo_rate_supported() || !imu_kx122_odr_supported()) {
        m0_error_set(0x20); // KX122 FIFO/ODR error
    }
#endif
#ifdef IMU_LSM6DS3_PRESENT
    if (!imu_lsm6ds3_fifo_rate_supported() || !imu_lsm6ds3_odr_supported()) {
        m0_error_set(0x40); // LSM6DS3 FIFO/ODR error
    }
#endif
#ifdef IMU_LSM303C_PRESENT
    if (!imu_lsm303c_fifo_rate_supported() || !imu_lsm303c_odr_supported()) {
        m0_error_set(0x80); // LSM303C FIFO/ODR error
    }
#endif

    // RF desync
    if (rf_monitor_get_errors() > 10) m0_error_set(0x04);

    // GPS UART framing
    char nmea[128];
    if (gps_uart_read_line(nmea, sizeof(nmea)) < 0) m0_error_set(0x08);

    // RAM overflow
    if (ram_monitor_get_free() < 1024) m0_error_set(0x10);

    // Core starvation (PID delay already handled)
}
