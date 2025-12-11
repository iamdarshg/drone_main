// imu_monitor.c - IMU subsystem error and status monitoring
#include "imu_monitor.h"
#include "imu_kx122.h"
#include "imu_lsm6ds3.h"
#include "imu_lsm303c.h"
#include <stdint.h>

static volatile uint32_t imu_error_flags = 0;
static volatile uint32_t imu_status_flags = 0;

void imu_monitor_check_all(void) {
    float result;
    if (imu_kx122_selftest(&result) != 0 || result < 0.5f) imu_error_flags |= 0x01;
    if (imu_lsm6ds3_selftest(&result) != 0 || result < 0.5f) imu_error_flags |= 0x02;
    if (imu_lsm303c_selftest(&result) != 0 || result < 0.5f) imu_error_flags |= 0x04;
}

void imu_monitor_clear_errors(void) { imu_error_flags = 0; }
uint32_t imu_monitor_get_errors(void) { return imu_error_flags; }
uint32_t imu_monitor_get_status(void) { return imu_status_flags; }
