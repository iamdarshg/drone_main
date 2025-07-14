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

#endif
