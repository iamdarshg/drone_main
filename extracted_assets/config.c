/*
 * ENHANCED config.c - Drop-in replacement for firmware/Config/config.c
 * Adds flash protection and crash detection state management
 */
#include "config.h"
#include "flash.h"
#include "logging.h"
#include <string.h>

#define CONFIG_FLASH_OFFSET 0
#define FLASH_MAGIC_NUMBER 0xDEADBEEF
#define CONFIG_VERSION 1

// Enhanced config structure with protection
typedef struct {
    uint32_t magic_number;
    uint32_t version;
    uint32_t checksum;
    config_params_t config_primary;
    config_params_t config_backup;
    uint32_t write_count;
    
    // NEW: Crash detection data
    bool crash_detected;
    float last_known_lat;
    float last_known_lon;
    uint32_t crash_timestamp;
    uint8_t crash_reason;
} protected_config_t;

static uint32_t calculate_checksum(const config_params_t *config) {
    uint32_t checksum = 0;
    const uint8_t *data = (const uint8_t*)config;
    for (int i = 0; i < sizeof(*config); i++) {
        checksum += data[i];
    }
    return checksum;
}

// ENHANCED: Protected config loading with crash detection
int config_load(config_params_t *cfg) {
    if (!cfg) return -1;
    
    protected_config_t protected_config;
    
    if (flash_read(CONFIG_FLASH_OFFSET, &protected_config, sizeof(protected_config)) != 0) {
        log_error("Flash read failed - using defaults");
        config_set_defaults(cfg);
        return -1;
    }
    
    // Validate header
    if (protected_config.magic_number != FLASH_MAGIC_NUMBER ||
        protected_config.version != CONFIG_VERSION) {
        log_error("Config header invalid - using defaults");
        config_set_defaults(cfg);
        return -1;
    }
    
    // Check for crash data
    if (protected_config.crash_detected) {
        log_error("CRASH DETECTED ON LAST FLIGHT!");
        log_error("Last position: %.6f, %.6f", 
                 protected_config.last_known_lat, protected_config.last_known_lon);
        log_error("Crash reason: %d, timestamp: %lu", 
                 protected_config.crash_reason, protected_config.crash_timestamp);
        
        // Clear crash flag for next flight
        protected_config.crash_detected = false;
        flash_erase();
        flash_write(CONFIG_FLASH_OFFSET, &protected_config, sizeof(protected_config));
    }
    
    // Validate checksums
    uint32_t primary_checksum = calculate_checksum(&protected_config.config_primary);
    uint32_t backup_checksum = calculate_checksum(&protected_config.config_backup);
    
    if (primary_checksum == protected_config.checksum) {
        *cfg = protected_config.config_primary;
        log_info("Config loaded from primary (writes: %lu)", protected_config.write_count);
        return 0;
    } else if (backup_checksum == protected_config.checksum) {
        *cfg = protected_config.config_backup;
        log_warning("Primary corrupted, using backup");
        return 0;
    } else {
        log_error("Both configs corrupted - using defaults");
        config_set_defaults(cfg);
        return -1;
    }
}

// ENHANCED: Protected config saving
int config_save(const config_params_t *cfg) {
    if (!cfg) return -1;
    
    protected_config_t protected_config = {0};
    
    // Read existing data to preserve crash info and write count
    flash_read(CONFIG_FLASH_OFFSET, &protected_config, sizeof(protected_config));
    
    protected_config.magic_number = FLASH_MAGIC_NUMBER;
    protected_config.version = CONFIG_VERSION;
    protected_config.config_primary = *cfg;
    protected_config.config_backup = *cfg;
    protected_config.checksum = calculate_checksum(cfg);
    protected_config.write_count++;
    
    flash_erase();
    
    if (flash_write(CONFIG_FLASH_OFFSET, &protected_config, sizeof(protected_config)) != 0) {
        log_error("Protected config write failed");
        return -1;
    }
    
    log_info("Protected config saved (write %lu)", protected_config.write_count);
    return 0;
}

// ENHANCED: Set defaults with new parameters
void config_set_defaults(config_params_t *cfg) {
    if (!cfg) return;
    
    // Existing defaults
    cfg->pid_kp = 1.0f;
    cfg->pid_ki = 0.1f;
    cfg->pid_kd = 0.05f;
    cfg->kalman_q = 0.01f;
    cfg->kalman_r = 0.1f;
    
    // NEW: Enhanced flight controller defaults
    cfg->roll_kp = 4.0f;
    cfg->roll_ki = 0.1f;
    cfg->roll_kd = 0.2f;
    cfg->pitch_kp = 4.0f;
    cfg->pitch_ki = 0.1f;
    cfg->pitch_kd = 0.2f;
    cfg->yaw_kp = 2.0f;
    cfg->yaw_ki = 0.05f;
    cfg->yaw_kd = 0.0f;
    cfg->max_angle = 0.52f;    // 30 degrees
    cfg->max_rate = 3.14f;     // 180 deg/s
    
    // Motor defaults
    cfg->motor_min_pwm = 1100;
    cfg->motor_max_pwm = 1900;
    cfg->motor_idle_pwm = 1050;
    
    // Telemetry defaults
    cfg->telemetry_rate_hz = 20;
    cfg->telemetry_enable_attitude = 1;
    cfg->telemetry_enable_motors = 1;
}

// NEW: Crash detection functions
void config_save_crash_data(float lat, float lon, uint8_t reason) {
    protected_config_t protected_config = {0};
    
    // Read existing config
    flash_read(CONFIG_FLASH_OFFSET, &protected_config, sizeof(protected_config));
    
    // Add crash data
    protected_config.crash_detected = true;
    protected_config.last_known_lat = lat;
    protected_config.last_known_lon = lon;
    protected_config.crash_timestamp = 0; // Would use RTC timestamp
    protected_config.crash_reason = reason;
    
    // Save immediately
    flash_erase();
    flash_write(CONFIG_FLASH_OFFSET, &protected_config, sizeof(protected_config));
    
    log_error("CRASH DATA SAVED: %.6f, %.6f, reason %d", lat, lon, reason);
}

bool config_has_crash_data(void) {
    protected_config_t protected_config;
    if (flash_read(CONFIG_FLASH_OFFSET, &protected_config, sizeof(protected_config)) != 0) {
        return false;
    }
    
    return (protected_config.magic_number == FLASH_MAGIC_NUMBER && 
            protected_config.crash_detected);
}