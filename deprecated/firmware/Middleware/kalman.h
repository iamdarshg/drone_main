#ifndef KALMAN_H
#define KALMAN_H
#include <stddef.h>
// Simple 1D Kalman filter struct
typedef struct {
    float q; // process noise covariance
    float r; // measurement noise covariance
    float x; // value
    float p; // estimation error covariance
    float k; // kalman gain
} kalman1d_t;

void kalman1d_init(kalman1d_t *kf, float q, float r, float x0, float p0);
float kalman1d_update(kalman1d_t *kf, float measurement);

// Multi-dimensional fusion (for 3D accel/gyro/mag)
void kalman3d_init(kalman1d_t kf[3], float q, float r, const float *x0, float p0);
void kalman3d_update(kalman1d_t kf[3], const float *meas, float *out);

// 9-DOF Kalman filter for accel/gyro/mag fusion
// State: [x, y, z, vx, vy, vz, ax, ay, az, ...] (expand as needed)
typedef struct {
    float state[9];      // [ax, ay, az, gx, gy, gz, mx, my, mz]
    float P[9][9];       // Covariance matrix
    float Q[9][9];       // Process noise
    float R[9][9];       // Measurement noise
    float K[9][9];       // Kalman gain
} kalman9dof_t;

void kalman9dof_init(kalman9dof_t *kf, const float *init_state, const float Q[9][9], const float R[9][9]);
void kalman9dof_update(kalman9dof_t *kf, const float meas[9]);

#endif
