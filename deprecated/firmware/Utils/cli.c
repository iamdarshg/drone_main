// cli.c - Simple CLI for runtime PID/Kalman tuning
#include "cli.h"
#include "Config/config.h"
#include <stdio.h>
#include <string.h>

// Example: parse "set pid kp 1.2" or "set kalman q 0.01"
int cli_pid_kalman_tune(const char *cmdline) {
    config_params_t cfg;
    if (config_load(&cfg) != 0) config_set_defaults(&cfg);
    char param[16];
    float value;
    if (sscanf(cmdline, "set pid kp %f", &value) == 1) {
        cfg.pid_kp = value;
    } else if (sscanf(cmdline, "set pid ki %f", &value) == 1) {
        cfg.pid_ki = value;
    } else if (sscanf(cmdline, "set pid kd %f", &value) == 1) {
        cfg.pid_kd = value;
    } else if (sscanf(cmdline, "set kalman q %f", &value) == 1) {
        cfg.kalman_q = value;
    } else if (sscanf(cmdline, "set kalman r %f", &value) == 1) {
        cfg.kalman_r = value;
    } else {
        printf("Unknown command\n");
        return -1;
    }
    config_save(&cfg);
    printf("Config updated and saved.\n");
    return 0;
}
