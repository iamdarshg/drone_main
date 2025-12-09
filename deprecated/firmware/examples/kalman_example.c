// Example: Using the Kalman filter for 1D sensor fusion
#include "kalman.h"
#include <stdio.h>

int main(void) {
    kalman1d_t kf;
    kalman1d_init(&kf, 0.0f, 1.0f, 0.01f, 0.1f);
    float measurement = 1.0f;
    for (int i = 0; i < 10; ++i) {
        float estimate = kalman1d_update(&kf, measurement);
        printf("Estimate[%d]: %f\n", i, estimate);
    }
    return 0;
}
