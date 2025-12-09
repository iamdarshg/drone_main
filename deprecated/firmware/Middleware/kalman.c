#include "kalman.h"
#include <string.h>
#include <stdio.h>

// 1D Kalman filter implementation
void kalman1d_init(kalman1d_t *kf, float q, float r, float x0, float p0) {
    kf->q = q;
    kf->r = r;
    kf->x = x0;
    kf->p = p0;
    kf->k = 0.0f;
}
float kalman1d_update(kalman1d_t *kf, float measurement) {
    // Prediction update
    kf->p += kf->q;
    // Measurement update
    kf->k = kf->p / (kf->p + kf->r);
    kf->x += kf->k * (measurement - kf->x);
    kf->p *= (1.0f - kf->k);
    return kf->x;
}
void kalman3d_init(kalman1d_t kf[3], float q, float r, const float *x0, float p0) {
    for (int i = 0; i < 3; ++i) kalman1d_init(&kf[i], q, r, x0 ? x0[i] : 0.0f, p0);
}
void kalman3d_update(kalman1d_t kf[3], const float *meas, float *out) {
    for (int i = 0; i < 3; ++i) out[i] = kalman1d_update(&kf[i], meas[i]);
}

void kalman9dof_init(kalman9dof_t *kf, const float *init_state, const float Q[9][9], const float R[9][9]) {
    if (init_state) memcpy(kf->state, init_state, sizeof(float)*9);
    else memset(kf->state, 0, sizeof(float)*9);
    memcpy(kf->Q, Q, sizeof(float)*81);
    memcpy(kf->R, R, sizeof(float)*81);
    memset(kf->P, 0, sizeof(float)*81);
    memset(kf->K, 0, sizeof(float)*81);
}

// Simple 9-DOF Kalman update (predict + correct, no control input)
void kalman9dof_update(kalman9dof_t *kf, const float meas[9]) {
    // Prediction: P = P + Q
    for (int i = 0; i < 9; ++i)
        for (int j = 0; j < 9; ++j)
            kf->P[i][j] += kf->Q[i][j];
    // Compute Kalman gain: K = P * inv(P + R) (diagonal only for simplicity)
    for (int i = 0; i < 9; ++i) {
        float denom = kf->P[i][i] + kf->R[i][i];
        kf->K[i][i] = denom > 0 ? kf->P[i][i] / denom : 0.0f;
    }
    // Update state: x = x + K*(z-x)
    for (int i = 0; i < 9; ++i)
        kf->state[i] += kf->K[i][i] * (meas[i] - kf->state[i]);
    // Update covariance: P = (I-K)P
    for (int i = 0; i < 9; ++i)
        for (int j = 0; j < 9; ++j)
            kf->P[i][j] *= (i == j) ? (1.0f - kf->K[i][i]) : 1.0f;
}
