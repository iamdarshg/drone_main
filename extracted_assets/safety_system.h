/*
 * safety_system.h - NEW FILE to add to firmware/Middleware/
 * Header for comprehensive safety system
 */
#ifndef SAFETY_SYSTEM_H
#define SAFETY_SYSTEM_H

#include <stdint.h>
#include <stdbool.h>
#include "Config/config.h"

// System degradation modes
typedef enum {
    DEGRADATION_NONE = 0,
    DEGRADATION_NO_MAG = 1,        // Lost magnetometer
    DEGRADATION_SINGLE_ACCEL = 2,  // Lost one accelerometer
    DEGRADATION_POOR_RF = 3,       // Poor RF link
    DEGRADATION_NO_GPS = 4,        // Lost GPS
    DEGRADATION_CRITICAL = 5       // Multiple failures
} system_degradation_t;

// Enhanced IMU data structure
typedef struct {
    float accel_x, accel_y, accel_z;        // Primary accelerometer (KX122)
    float accel_lsm_x, accel_lsm_y, accel_lsm_z; // Backup accelerometer (LSM6DS3)
    float gyro_x, gyro_y, gyro_z;           // Gyroscope (LSM6DS3)
    float mag_x, mag_y, mag_z;              // Magnetometer (LSM303C)
    float temp_kx122;                       // KX122 temperature
    float temp_lsm;                         // LSM temperature
    int valid_sensors;                      // Number of working sensors
} safety_imu_data_t;

// Main safety system functions
void safety_system_init(void);
void safety_system_update(void);
bool safety_is_system_healthy(void);
system_degradation_t safety_get_degradation_mode(void);

// I2C bus recovery
int safety_i2c_recovery(void);
int safety_i2c_transaction(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, uint8_t len, bool read);

// IMU sensor voting and validation
int safety_read_all_imus(safety_imu_data_t *imu_data);
bool safety_validate_imu(const safety_imu_data_t *imu_data);

// RF communication with retry
int safety_rf_send_with_retry(const uint8_t *data, size_t length);

// GPS and crash detection
void safety_update_gps_data(void);
void safety_detect_crash(void);
void safety_trigger_crash_detection(crash_reason_t reason);
void safety_send_emergency_telemetry(void);

// Memory protection
void* safety_malloc(size_t size);
void safety_free(void *ptr);

// Graceful degradation
void safety_update_degradation_mode(void);

#endif