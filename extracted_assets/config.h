// ENHANCED config.h - Drop-in replacement for firmware/Config/config.h
// Adds crash detection and enhanced safety parameters

#ifndef CONFIG_H
#define CONFIG_H
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float pid_kp;
    float pid_ki;
    float pid_kd;
    float kalman_q;
    float kalman_r;
    
    // Flight controller config
    float roll_kp;
    float roll_ki;
    float roll_kd;
    float pitch_kp;
    float pitch_ki;
    float pitch_kd;
    float yaw_kp;
    float yaw_ki;
    float yaw_kd;
    float max_angle;
    float max_rate;
    
    // Motor config
    uint16_t motor_min_pwm;
    uint16_t motor_max_pwm;
    uint16_t motor_idle_pwm;
    
    // Telemetry config
    uint16_t telemetry_rate_hz;
    int telemetry_enable_attitude;
    int telemetry_enable_motors;
} config_params_t;

// Crash detection reasons
typedef enum {
    CRASH_REASON_UNKNOWN = 0,
    CRASH_REASON_IMU_FAILURE = 1,
    CRASH_REASON_RF_TIMEOUT = 2,
    CRASH_REASON_BATTERY_LOW = 3,
    CRASH_REASON_MOTOR_FAILURE = 4,
    CRASH_REASON_THERMAL_SHUTDOWN = 5,
    CRASH_REASON_GPS_LOST = 6,
    CRASH_REASON_MANUAL_CRASH_DETECTION = 7
} crash_reason_t;

// Load config from flash (or other storage) - ENHANCED
int config_load(config_params_t *cfg);
// Save config to flash (or other storage) - ENHANCED
int config_save(const config_params_t *cfg);
// Set default config values - ENHANCED
void config_set_defaults(config_params_t *cfg);

// NEW: Crash detection functions
void config_save_crash_data(float lat, float lon, uint8_t reason);
bool config_has_crash_data(void);

#endif