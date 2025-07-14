// kalman9dof_example.c - Example: 9-DOF Kalman filter for accel/gyro/mag fusion
#include "kalman.h"
#include <stdio.h>
#include <string.h>

int main(void) {
    kalman9dof_t kf;
    float init_state[9] = {0};
    float Q[9][9] = {0}, R[9][9] = {0};
    for (int i = 0; i < 9; ++i) Q[i][i] = 0.01f, R[i][i] = 0.1f;
    kalman9dof_init(&kf, init_state, Q, R);
    float meas[9] = {1.0f, 0.5f, -0.2f, 0.01f, 0.02f, 0.03f, 0.3f, 0.2f, 0.1f};
    for (int step = 0; step < 10; ++step) {
        for (int i = 0; i < 9; ++i) meas[i] += 0.01f * (float)step;
        kalman9dof_update(&kf, meas);
        printf("Step %d: State=", step);
        for (int i = 0; i < 9; ++i) printf(" %.3f", kf.state[i]);
        printf("\n");
    }
    return 0;
}
