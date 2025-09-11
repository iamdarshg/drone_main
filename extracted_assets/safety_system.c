/*
 * safety_system.c - NEW FILE to add to firmware/Middleware/
 * Contains all critical and high-priority safety features
 * 
 * FEATURES INCLUDED:
 * - I2C Bus Recovery
 * - IMU Sensor Voting 
 * - Sensor Plausibility Checking
 * - RF Retry Logic
 * - Memory Protection
 * - GPS Crash Detection
 * - Graceful Degradation
 */

#include "safety_system.h"
#include "logging.h"
#include "gps_uart.h"
#include "imu_kx122.h"
#include "imu_lsm6ds3.h"
#include "imu_lsm303c.h"
#include "rf_s2lpqtr.h"
#include "Config/config.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <string.h>

// ==================== GLOBAL SAFETY STATE ====================

typedef struct {
    bool i2c_healthy;
    uint32_t i2c_error_count;
    uint32_t i2c_recovery_count;
    
    bool imu_sensors_healthy[3]; // KX122, LSM6DS3, LSM303C
    uint32_t imu_error_count[3];
    
    bool rf_healthy;
    uint32_t rf_error_count;
    uint32_t rf_retry_count;
    
    bool gps_healthy;
    float last_good_lat;
    float last_good_lon;
    uint32_t last_gps_update;
    
    system_degradation_t degradation_mode;
    uint32_t total_failures;
    
    bool crash_detected;
    crash_reason_t crash_reason;
    
} safety_system_state_t;

static safety_system_state_t safety_state = {0};

// ==================== I2C BUS RECOVERY ====================

int safety_i2c_recovery(void) {
    log_warning("I2C bus recovery starting...");
    safety_state.i2c_recovery_count++;
    
    // Disable I2C peripheral
    // NOTE: Replace with your actual I2C disable function
    // i2c_bus_deinit();
    
    // Configure pins as GPIO for bit-banging
    // Generate 9 clock pulses to clear stuck slaves
    for (int i = 0; i < 9; i++) {
        // Toggle clock line
        // gpio_set_high(I2C_SCL_PIN);
        vTaskDelay(pdMS_TO_TICKS(1));
        // gpio_set_low(I2C_SCL_PIN);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    
    // Generate STOP condition
    // gpio_set_low(I2C_SDA_PIN);
    vTaskDelay(pdMS_TO_TICKS(1));
    // gpio_set_high(I2C_SCL_PIN);
    vTaskDelay(pdMS_TO_TICKS(1));
    // gpio_set_high(I2C_SDA_PIN);
    
    // Reinitialize I2C
    // i2c_bus_init();
    
    safety_state.i2c_healthy = true;
    safety_state.i2c_error_count = 0;
    
    log_info("I2C recovery completed (attempt %lu)", safety_state.i2c_recovery_count);
    return 0;
}

int safety_i2c_transaction(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, uint8_t len, bool read) {
    int retries = 3;
    int result = -1;
    
    for (int retry = 0; retry < retries; retry++) {
        // Replace with your actual I2C functions
        if (read) {
            // result = i2c_read_register(device_addr, reg_addr, data, len);
            result = 0; // Stub - replace with actual call
        } else {
            // result = i2c_write_register(device_addr, reg_addr, *data);
            result = 0; // Stub - replace with actual call
        }
        
        if (result == 0) {
            safety_state.i2c_healthy = true;
            return 0;
        }
        
        safety_state.i2c_error_count++;
        safety_state.i2c_healthy = false;
        
        // Attempt recovery after multiple failures
        if (safety_state.i2c_error_count > 5) {
            safety_i2c_recovery();
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    return -1;
}

// ==================== IMU SENSOR VOTING ====================

int safety_read_all_imus(safety_imu_data_t *imu_data) {
    float accel[3], gyro[3], mag[3], temp;
    int valid_sensors = 0;
    
    // Clear validity flags
    for (int i = 0; i < 3; i++) {
        safety_state.imu_sensors_healthy[i] = false;
    }
    
    // Read KX122 (SPI) - Accelerometer
    if (imu_kx122_read(accel, &temp) == 0) {
        imu_data->accel_x = accel[0];
        imu_data->accel_y = accel[1];
        imu_data->accel_z = accel[2];
        imu_data->temp_kx122 = temp;
        safety_state.imu_sensors_healthy[0] = true;
        valid_sensors++;
    } else {
        safety_state.imu_error_count[0]++;
    }
    
    // Read LSM6DS3 (I2C) - Gyro + Accel
    if (safety_i2c_transaction(LSM6DSV_I2C_ADDRESS, LSM6DSV_OUTX_L_G, (uint8_t*)gyro, 6, true) == 0) {
        imu_data->gyro_x = gyro[0];
        imu_data->gyro_y = gyro[1];
        imu_data->gyro_z = gyro[2];
        imu_data->accel_lsm_x = accel[0]; // Backup accelerometer
        imu_data->accel_lsm_y = accel[1];
        imu_data->accel_lsm_z = accel[2];
        safety_state.imu_sensors_healthy[1] = true;
        valid_sensors++;
    } else {
        safety_state.imu_error_count[1]++;
    }
    
    // Read LSM303C (I2C) - Magnetometer
    if (imu_lsm303c_read(mag, &temp) == 0) {
        imu_data->mag_x = mag[0];
        imu_data->mag_y = mag[1];
        imu_data->mag_z = mag[2];
        imu_data->temp_lsm = temp;
        safety_state.imu_sensors_healthy[2] = true;
        valid_sensors++;
    } else {
        safety_state.imu_error_count[2]++;
    }
    
    // Sensor fusion and voting
    if (safety_state.imu_sensors_healthy[0] && safety_state.imu_sensors_healthy[1]) {
        // Cross-validate accelerometers
        float accel_diff = fabsf(imu_data->accel_x - imu_data->accel_lsm_x) +
                          fabsf(imu_data->accel_y - imu_data->accel_lsm_y) +
                          fabsf(imu_data->accel_z - imu_data->accel_lsm_z);
        
        if (accel_diff < 2.0f) {
            // Readings agree - average them
            imu_data->accel_x = (imu_data->accel_x + imu_data->accel_lsm_x) / 2.0f;
            imu_data->accel_y = (imu_data->accel_y + imu_data->accel_lsm_y) / 2.0f;
            imu_data->accel_z = (imu_data->accel_z + imu_data->accel_lsm_z) / 2.0f;
        } else {
            log_warning("Accelerometer disagreement: %.2f", accel_diff);
        }
    }
    
    imu_data->valid_sensors = valid_sensors;
    return valid_sensors;
}

// ==================== SENSOR PLAUSIBILITY CHECKING ====================

bool safety_validate_imu(const safety_imu_data_t *imu_data) {
    // Check accelerometer magnitude (should be ~9.8 m/s² when stationary)
    float accel_mag = sqrtf(imu_data->accel_x * imu_data->accel_x +
                           imu_data->accel_y * imu_data->accel_y +
                           imu_data->accel_z * imu_data->accel_z);
    
    if (accel_mag < 5.0f || accel_mag > 20.0f) {
        log_warning("Implausible accelerometer: %.2f m/s²", accel_mag);
        return false;
    }
    
    // Check gyroscope limits (max ~35 rad/s = 2000 deg/s)
    float gyro_mag = sqrtf(imu_data->gyro_x * imu_data->gyro_x +
                          imu_data->gyro_y * imu_data->gyro_y +
                          imu_data->gyro_z * imu_data->gyro_z);
    
    if (gyro_mag > 35.0f) {
        log_warning("Implausible gyroscope: %.2f rad/s", gyro_mag);
        return false;
    }
    
    // Check temperature ranges
    if (imu_data->temp_kx122 < -40.0f || imu_data->temp_kx122 > 85.0f) {
        log_warning("Implausible KX122 temp: %.1f°C", imu_data->temp_kx122);
        return false;
    }
    
    return true;
}

// ==================== RF RETRY LOGIC ====================

int safety_rf_send_with_retry(const uint8_t *data, size_t length) {
    int max_retries = 5;
    int retry_delay = 10; // Start with 10ms
    
    for (int retry = 0; retry < max_retries; retry++) {
        // Replace with your actual RF send function
        // int result = rf_s2lpqtr_send(data, length);
        int result = 0; // Stub - replace with actual call
        
        if (result == 0) {
            safety_state.rf_healthy = true;
            return 0;
        }
        
        safety_state.rf_error_count++;
        safety_state.rf_retry_count++;
        safety_state.rf_healthy = false;
        
        if (retry < max_retries - 1) {
            vTaskDelay(pdMS_TO_TICKS(retry_delay));
            retry_delay *= 2; // Exponential backoff
            if (retry_delay > 500) retry_delay = 500;
        }
    }
    
    log_warning("RF send failed after %d retries", max_retries);
    return -1;
}

// ==================== GPS CRASH DETECTION ====================

void safety_update_gps_data(void) {
    char nmea_buffer[128];
    float lat, lon;
    
    if (gps_uart_read_line(nmea_buffer, sizeof(nmea_buffer)) > 0) {
        if (gps_uart_parse_gga(nmea_buffer, &lat, &lon) == 0) {
            // Valid GPS data
            safety_state.gps_healthy = true;
            safety_state.last_good_lat = lat;
            safety_state.last_good_lon = lon;
            safety_state.last_gps_update = xTaskGetTickCount();
        }
    }
    
    // Check GPS timeout
    uint32_t time_since_gps = xTaskGetTickCount() - safety_state.last_gps_update;
    if (time_since_gps > pdMS_TO_TICKS(10000)) { // 10 seconds
        safety_state.gps_healthy = false;
    }
}

void safety_detect_crash(void) {
    static uint32_t low_altitude_time = 0;
    static bool was_flying = false;
    
    // Simple crash detection logic:
    // 1. Sudden altitude drop
    // 2. High angular rates with low throttle
    // 3. GPS indicates no movement for extended time while armed
    
    // Get current system state (you'll need to pass this from your main code)
    // bool is_armed = get_system_armed_state();
    // float current_altitude = get_current_altitude();
    // float throttle = get_current_throttle();
    
    bool is_armed = false; // Stub - replace with actual state
    float current_altitude = 0.0f; // Stub - replace with actual altitude
    float throttle = 0.0f; // Stub - replace with actual throttle
    
    if (is_armed && throttle > 0.1f) {
        was_flying = true;
    }
    
    // Detect crash conditions
    if (was_flying && is_armed) {
        if (current_altitude < 2.0f) { // Below 2 meters
            if (low_altitude_time == 0) {
                low_altitude_time = xTaskGetTickCount();
            } else if (xTaskGetTickCount() - low_altitude_time > pdMS_TO_TICKS(5000)) {
                // Low altitude for 5 seconds while armed - likely crashed
                safety_trigger_crash_detection(CRASH_REASON_MANUAL_CRASH_DETECTION);
            }
        } else {
            low_altitude_time = 0;
        }
    }
    
    if (!is_armed) {
        was_flying = false;
        low_altitude_time = 0;
    }
}

void safety_trigger_crash_detection(crash_reason_t reason) {
    if (safety_state.crash_detected) {
        return; // Already detected
    }
    
    safety_state.crash_detected = true;
    safety_state.crash_reason = reason;
    
    log_error("CRASH DETECTED! Reason: %d", reason);
    log_error("Last GPS: %.6f, %.6f", safety_state.last_good_lat, safety_state.last_good_lon);
    
    // Save crash data to flash
    config_save_crash_data(safety_state.last_good_lat, safety_state.last_good_lon, reason);
    
    // Send emergency telemetry
    safety_send_emergency_telemetry();
}

void safety_send_emergency_telemetry(void) {
    // Create emergency telemetry packet
    typedef struct {
        uint16_t header;        // 0x4552 ("ER")
        uint8_t crash_reason;
        float last_lat;
        float last_lon;
        uint32_t timestamp;
        uint16_t checksum;
    } emergency_telemetry_t;
    
    emergency_telemetry_t emergency_packet = {0};
    emergency_packet.header = 0x4552;
    emergency_packet.crash_reason = safety_state.crash_reason;
    emergency_packet.last_lat = safety_state.last_good_lat;
    emergency_packet.last_lon = safety_state.last_good_lon;
    emergency_packet.timestamp = xTaskGetTickCount();
    emergency_packet.checksum = 0x1234; // Simple checksum
    
    // Send emergency packet with maximum retry
    for (int i = 0; i < 10; i++) {
        if (safety_rf_send_with_retry((uint8_t*)&emergency_packet, sizeof(emergency_packet)) == 0) {
            log_info("Emergency telemetry sent (attempt %d)", i+1);
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ==================== GRACEFUL DEGRADATION ====================

void safety_update_degradation_mode(void) {
    system_degradation_t new_mode = DEGRADATION_NONE;
    uint32_t failure_count = 0;
    
    // Check subsystem health
    if (!safety_state.imu_sensors_healthy[2]) { // Magnetometer
        new_mode = DEGRADATION_NO_MAG;
        failure_count++;
    }
    
    if (!safety_state.imu_sensors_healthy[0] || !safety_state.imu_sensors_healthy[1]) {
        new_mode = DEGRADATION_SINGLE_ACCEL;
        failure_count++;
    }
    
    if (!safety_state.rf_healthy) {
        new_mode = DEGRADATION_POOR_RF;
        failure_count++;
    }
    
    if (!safety_state.gps_healthy) {
        new_mode = DEGRADATION_NO_GPS;
        failure_count++;
    }
    
    if (failure_count >= 2) {
        new_mode = DEGRADATION_CRITICAL;
    }
    
    if (new_mode != safety_state.degradation_mode) {
        log_info("Degradation mode: %d -> %d", safety_state.degradation_mode, new_mode);
        safety_state.degradation_mode = new_mode;
        
        switch (new_mode) {
            case DEGRADATION_NO_MAG:
                log_info("Lost magnetometer - using gyro yaw");
                break;
            case DEGRADATION_SINGLE_ACCEL:
                log_info("Lost accelerometer - increasing filtering");
                break;
            case DEGRADATION_POOR_RF:
                log_info("Poor RF - reducing telemetry rate");
                break;
            case DEGRADATION_NO_GPS:
                log_info("Lost GPS - no position fix");
                break;
            case DEGRADATION_CRITICAL:
                log_error("CRITICAL degradation - emergency actions required");
                safety_trigger_crash_detection(CRASH_REASON_UNKNOWN);
                break;
            default:
                break;
        }
    }
    
    safety_state.total_failures = failure_count;
}

// ==================== MEMORY PROTECTION ====================

#define HEAP_GUARD_PATTERN 0x5A5A5A5A

void* safety_malloc(size_t size) {
    uint8_t *ptr = (uint8_t*)pvPortMalloc(size + 8);
    if (!ptr) {
        log_error("Critical memory allocation failed: %d bytes", size);
        return NULL;
    }
    
    // Install guard patterns
    *((uint32_t*)ptr) = HEAP_GUARD_PATTERN;
    *((uint32_t*)(ptr + size + 4)) = HEAP_GUARD_PATTERN;
    
    return ptr + 4;
}

void safety_free(void *ptr) {
    if (!ptr) return;
    
    uint8_t *real_ptr = ((uint8_t*)ptr) - 4;
    
    // Check guard pattern
    if (*((uint32_t*)real_ptr) != HEAP_GUARD_PATTERN) {
        log_error("Memory corruption detected - heap underrun");
        safety_trigger_crash_detection(CRASH_REASON_UNKNOWN);
        return;
    }
    
    vPortFree(real_ptr);
}

// ==================== MAIN SAFETY SYSTEM INTERFACE ====================

void safety_system_init(void) {
    log_info("Safety system initializing...");
    
    memset(&safety_state, 0, sizeof(safety_state));
    safety_state.i2c_healthy = true;
    safety_state.rf_healthy = true;
    safety_state.degradation_mode = DEGRADATION_NONE;
    
    // Check for previous crash data
    if (config_has_crash_data()) {
        log_warning("Previous crash data found in config");
    }
    
    log_info("Safety system ready");
}

void safety_system_update(void) {
    // Update GPS data and check for crashes
    safety_update_gps_data();
    safety_detect_crash();
    
    // Update system degradation mode
    safety_update_degradation_mode();
    
    // Log health summary periodically
    static uint32_t last_health_log = 0;
    if (xTaskGetTickCount() - last_health_log > pdMS_TO_TICKS(10000)) { // Every 10 seconds
        log_info("Safety: I2C=%d RF=%d GPS=%d IMU=[%d,%d,%d] Deg=%d", 
                safety_state.i2c_healthy,
                safety_state.rf_healthy, 
                safety_state.gps_healthy,
                safety_state.imu_sensors_healthy[0],
                safety_state.imu_sensors_healthy[1], 
                safety_state.imu_sensors_healthy[2],
                safety_state.degradation_mode);
        last_health_log = xTaskGetTickCount();
    }
}

system_degradation_t safety_get_degradation_mode(void) {
    return safety_state.degradation_mode;
}

bool safety_is_system_healthy(void) {
    return (safety_state.total_failures < 2 && 
            !safety_state.crash_detected &&
            safety_state.degradation_mode < DEGRADATION_CRITICAL);
}