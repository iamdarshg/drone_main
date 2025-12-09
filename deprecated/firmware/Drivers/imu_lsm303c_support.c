// imu_lsm303c_support.c - Check if ODR/FIFO settings are supported by hardware
#include "imu_lsm303c.h"
#include <stdbool.h>

// Reference: LSM303C datasheet
static const uint8_t supported_accel_odrs[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07}; // Example: 10Hz-1.344kHz
static const uint8_t supported_mag_odrs[] = {0x00, 0x01, 0x02, 0x03}; // Example: 20, 25, 50, 100Hz
static const uint8_t supported_fifo_modes[] = {0x00, 0x01, 0x02, 0x03}; // Bypass, FIFO, Stream, Stream-to-FIFO

bool imu_lsm303c_is_accel_odr_supported(uint8_t odr) {
    for (unsigned i = 0; i < sizeof(supported_accel_odrs); ++i)
        if (supported_accel_odrs[i] == odr) return true;
    return false;
}
bool imu_lsm303c_is_mag_odr_supported(uint8_t odr) {
    for (unsigned i = 0; i < sizeof(supported_mag_odrs); ++i)
        if (supported_mag_odrs[i] == odr) return true;
    return false;
}
bool imu_lsm303c_is_fifo_mode_supported(uint8_t mode) {
    for (unsigned i = 0; i < sizeof(supported_fifo_modes); ++i)
        if (supported_fifo_modes[i] == mode) return true;
    return false;
}
