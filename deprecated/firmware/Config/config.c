// config.c - Implementation for PID and Kalman config
#include "config.h"
#include "flash.h"
#include <string.h>

#define CONFIG_FLASH_OFFSET 0

int config_load(config_params_t *cfg) {
    if (!cfg) return -1;
    if (flash_read(CONFIG_FLASH_OFFSET, cfg, sizeof(*cfg)) != 0) {
        config_set_defaults(cfg);
        return -1;
    }
    return 0;
}

int config_save(const config_params_t *cfg) {
    if (!cfg) return -1;
    flash_erase();
    return flash_write(CONFIG_FLASH_OFFSET, cfg, sizeof(*cfg));
}

void config_set_defaults(config_params_t *cfg) {
    if (!cfg) return;
    cfg->pid_kp = 1.0f;
    cfg->pid_ki = 0.1f;
    cfg->pid_kd = 0.05f;
    cfg->kalman_q = 0.01f;
    cfg->kalman_r = 0.1f;
}
