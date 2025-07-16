// kalman.h - Unscented Kalman Filter for 9DOF IMU (accel/gyro/mag)
#ifndef KALMAN_H
#define KALMAN_H

#include <stdint.h>
#include "matrix_math.h" // For matrix operations

typedef struct {
    float x[9];       // State vector [ax,ay,az,gx,gy,gz,mx,my,mz]
    float P[9][9];    // Covariance matrix
    float Q[9][9];    // Process noise
    float R[9][9];    // Measurement noise
    float dt;        // Time step
    float sigma;     // Sigma point scaling
} ukf9dof_t;

// Initialize UKF
void ukf9dof_init(ukf9dof_t *kf, 
                 const float *Q, const float *R, 
                 float dt, float sigma);

// Prediction step
void ukf9dof_predict(ukf9dof_t *kf);

// Update step
void ukf9dof_update(ukf9dof_t *kf, const float *z);

// Get current state estimate
void ukf9dof_get_state(ukf9dof_t *kf, float *state);

#endif // KALMAN_H
