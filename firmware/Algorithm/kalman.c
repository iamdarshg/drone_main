// kalman.c - Unscented Kalman Filter implementation for 9DOF IMU
#include "kalman.h"
#include <math.h>
#include <string.h>

// Sigma point weights
static float Wm[19];
static float Wc[19];

// Helper function for Cholesky decomposition
static void cholesky(const float A[9][9], float L[9][9]) {
    memset(L, 0, sizeof(float)*9*9);
    for(int i=0; i<9; i++) {
        for(int j=0; j<=i; j++) {
            float sum = 0;
            for(int k=0; k<j; k++) sum += L[i][k] * L[j][k];
            L[i][j] = (i == j) ? sqrtf(A[i][i] - sum) : 
                       (1.0/L[j][j] * (A[i][j] - sum));
        }
    }
}

// Initialize UKF
void ukf9dof_init(ukf9dof_t *kf, const float *Q, const float *R, 
                 float dt, float sigma) {
    // Initialize state and covariance
    memset(kf->x, 0, sizeof(kf->x));
    memset(kf->P, 0, sizeof(kf->P));
    for(int i=0; i<9; i++) kf->P[i][i] = 1.0f;
    
    // Copy noise matrices
    memcpy(kf->Q, Q, sizeof(kf->Q));
    memcpy(kf->R, R, sizeof(kf->R));
    
    kf->dt = dt;
    kf->sigma = sigma;
    
    // Calculate weights
    float lambda = sigma*sigma*9 - 9;
    float gamma = sqrtf(9 + lambda);
    
    Wm[0] = lambda / (9 + lambda);
    Wc[0] = Wm[0] + (1 - sigma*sigma + 2);
    for(int i=1; i<19; i++) {
        Wm[i] = 1.0 / (2*(9 + lambda));
        Wc[i] = Wm[i];
    }
}

// Prediction step
void ukf9dof_predict(ukf9dof_t *kf) {
    // Generate sigma points
    float X[19][9];
    float L[9][9];
    
    cholesky(kf->P, L);
    
    // First sigma point is current state
    memcpy(X[0], kf->x, sizeof(kf->x));
    
    // Generate remaining sigma points
    for(int i=0; i<9; i++) {
        for(int j=0; j<9; j++) {
            X[1+i][j] = kf->x[j] + kf->sigma * L[j][i];
            X[10+i][j] = kf->x[j] - kf->sigma * L[j][i];
        }
    }
    
    // Propagate sigma points through process model
    for(int i=0; i<19; i++) {
        // Simple constant velocity model for now
        // Can be replaced with more sophisticated IMU model
        for(int j=0; j<3; j++) {
            X[i][j] += X[i][3+j] * kf->dt; // Position update
        }
    }
    
    // Compute predicted state mean
    memset(kf->x, 0, sizeof(kf->x));
    for(int i=0; i<19; i++) {
        for(int j=0; j<9; j++) {
            kf->x[j] += Wm[i] * X[i][j];
        }
    }
    
    // Compute predicted covariance
    memset(kf->P, 0, sizeof(kf->P));
    for(int i=0; i<19; i++) {
        float dx[9];
        for(int j=0; j<9; j++) dx[j] = X[i][j] - kf->x[j];
        
        for(int j=0; j<9; j++) {
            for(int k=0; k<9; k++) {
                kf->P[j][k] += Wc[i] * dx[j] * dx[k];
            }
        }
    }
    
    // Add process noise
    for(int i=0; i<9; i++) {
        for(int j=0; j<9; j++) {
            kf->P[i][j] += kf->Q[i][j];
        }
    }
}

// Update step
void ukf9dof_update(ukf9dof_t *kf, const float *z) {
    // Similar sigma point generation as predict step
    float X[19][9], Z[19][9];
    float L[9][9];
    
    cholesky(kf->P, L);
    
    // First sigma point
    memcpy(X[0], kf->x, sizeof(kf->x));
    memcpy(Z[0], kf->x, sizeof(kf->x)); // Simple measurement model
    
    // Generate remaining sigma points
    for(int i=0; i<9; i++) {
        for(int j=0; j<9; j++) {
            X[1+i][j] = kf->x[j] + kf->sigma * L[j][i];
            X[10+i][j] = kf->x[j] - kf->sigma * L[j][i];
            
            // Measurement model (identity for now)
            Z[1+i][j] = X[1+i][j];
            Z[10+i][j] = X[10+i][j];
        }
    }
    
    // Compute predicted measurement mean
    float z_hat[9] = {0};
    for(int i=0; i<19; i++) {
        for(int j=0; j<9; j++) {
            z_hat[j] += Wm[i] * Z[i][j];
        }
    }
    
    // Compute innovation covariance
    float S[9][9] = {0};
    for(int i=0; i<19; i++) {
        float dz[9];
        for(int j=0; j<9; j++) dz[j] = Z[i][j] - z_hat[j];
        
        for(int j=0; j<9; j++) {
            for(int k=0; k<9; k++) {
                S[j][k] += Wc[i] * dz[j] * dz[k];
            }
        }
    }
    
    // Add measurement noise
    for(int i=0; i<9; i++) {
        for(int j=0; j<9; j++) {
            S[i][j] += kf->R[i][j];
        }
    }
    
    // Compute cross-covariance
    float Pxz[9][9] = {0};
    for(int i=0; i<19; i++) {
        float dx[9], dz[9];
        for(int j=0; j<9; j++) {
            dx[j] = X[i][j] - kf->x[j];
            dz[j] = Z[i][j] - z_hat[j];
        }
        
        for(int j=0; j<9; j++) {
            for(int k=0; k<9; k++) {
                Pxz[j][k] += Wc[i] * dx[j] * dz[k];
            }
        }
    }
    
    // Kalman gain
    float K[9][9];
    // Invert S (simplified for demo)
    for(int i=0; i<9; i++) {
        for(int j=0; j<9; j++) {
            K[i][j] = Pxz[i][j] / S[j][j]; // Simplified inversion
        }
    }
    
    // Update state
    float dz[9];
    for(int i=0; i<9; i++) dz[i] = z[i] - z_hat[i];
    
    for(int i=0; i<9; i++) {
        for(int j=0; j<9; j++) {
            kf->x[i] += K[i][j] * dz[j];
        }
    }
    
    // Update covariance
    float I[9][9] = {0};
    for(int i=0; i<9; i++) I[i][i] = 1.0f;
    
    for(int i=0; i<9; i++) {
        for(int j=0; j<9; j++) {
            for(int k=0; k<9; k++) {
                I[i][j] -= K[i][k] * S[k][j];
            }
        }
    }
    
    for(int i=0; i<9; i++) {
        for(int j=0; j<9; j++) {
            kf->P[i][j] = 0;
            for(int k=0; k<9; k++) {
                for(int l=0; l<9; l++) {
                    kf->P[i][j] += I[i][k] * Pxz[k][l] * I[j][l];
                }
            }
        }
    }
}

// Get current state estimate
void ukf9dof_get_state(ukf9dof_t *kf, float *state) {
    memcpy(state, kf->x, sizeof(kf->x));
}
