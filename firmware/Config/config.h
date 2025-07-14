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
} config_params_t;

// Load config from flash (or other storage)
int config_load(config_params_t *cfg);
// Save config to flash (or other storage)
int config_save(const config_params_t *cfg);
// Set default config values
void config_set_defaults(config_params_t *cfg);

#endif
