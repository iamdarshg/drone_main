#include "kalman.h"
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
