// tuning_loop_example.c - Example: PID and Kalman tuning loop updating config
#include "config.h"
#include "pid.h"
#include "kalman.h"
#include <stdio.h>

int main(void) {
    config_params_t cfg;
    if (config_load(&cfg) != 0) printf("Loaded defaults\n");
    printf("Initial PID: kp=%.2f ki=%.2f kd=%.2f\n", cfg.pid_kp, cfg.pid_ki, cfg.pid_kd);
    printf("Initial Kalman: q=%.3f r=%.3f\n", cfg.kalman_q, cfg.kalman_r);
    // Simulate tuning loop
    for (int i = 0; i < 5; ++i) {
        cfg.pid_kp += 0.1f;
        cfg.kalman_q *= 1.1f;
        config_save(&cfg);
        printf("Tuned PID: kp=%.2f\n", cfg.pid_kp);
        printf("Tuned Kalman: q=%.3f\n", cfg.kalman_q);
    }
    // Use tuned values
    pid_t pid;
    float sensors[3] = {0};
    pid_init(&pid, cfg.pid_kp, cfg.pid_ki, cfg.pid_kd, sensors, 3);
    kalman1d_t kf;
    kalman1d_init(&kf, cfg.kalman_q, cfg.kalman_r, 0.0f, 1.0f);
    return 0;
}
