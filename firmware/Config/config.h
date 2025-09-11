// config.h - Central configuration for PID and Kalman parameters
#ifndef CONFIG_H
#define CONFIG_H
#include <stdint.h>
#include <stddef.h>

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

// Load config from flash (or other storage)
int config_load(config_params_t *cfg);
// Save config to flash (or other storage)
int config_save(const config_params_t *cfg);
// Set default config values
void config_set_defaults(config_params_t *cfg);

#endif
