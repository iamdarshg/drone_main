// calibration_test_example.c - Example: Software calibration and test routines for Kalman and PID
#include "kalman.h"
#include "pid.h"
#include <stdio.h>
#include <math.h>

// Simulate a noisy sensor for Kalman filter calibration
typedef struct {
    float true_value;
    float noise_std;
} sensor_sim_t;

float sensor_sim_read(sensor_sim_t *sim) {
    // Simple noise: uniform [-noise_std, noise_std]
    float noise = ((float)rand()/(float)RAND_MAX) * 2.0f * sim->noise_std - sim->noise_std;
    return sim->true_value + noise;
}

void test_kalman_calibration(void) {
    kalman1d_t kf;
    kalman1d_init(&kf, 0.01f, 0.1f, 0.0f, 1.0f);
    sensor_sim_t sim = {1.0f, 0.2f};
    printf("Kalman calibration test:\n");
    for (int i = 0; i < 20; ++i) {
        float meas = sensor_sim_read(&sim);
        float est = kalman1d_update(&kf, meas);
        printf("Step %d: Meas=%.3f, Est=%.3f\n", i, meas, est);
    }
}

void test_pid_calibration(void) {
    float sensors[3] = {0.0f, 0.0f, 0.0f};
    pid_t pid;
    pid_init(&pid, 1.0f, 0.2f, 0.05f, sensors, 3);
    float setpoint = 5.0f;
    float dt = 0.05f;
    printf("PID calibration test:\n");
    for (int i = 0; i < 20; ++i) {
        float output = pid_update(&pid, setpoint, dt);
        printf("Step %d: Output=%.3f, Inputs=%.3f,%.3f,%.3f\n", i, output, sensors[0], sensors[1], sensors[2]);
        // Simulate system response
        for (int j = 0; j < 3; ++j) sensors[j] += output * 0.1f;
    }
}

int main(void) {
    test_kalman_calibration();
    test_pid_calibration();
    return 0;
}
