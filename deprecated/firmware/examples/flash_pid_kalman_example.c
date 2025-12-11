// flash_pid_kalman_example.c - Example: Store/load PID and Kalman variables in Flash
#include "flash.h"
#include "pid.h"
#include "kalman.h"
#include <stdio.h>

#define PID_FLASH_OFFSET     0
#define KALMAN_FLASH_OFFSET  128

int main(void) {
    // Example PID and Kalman variables
    pid_t pid = {1.0f, 0.1f, 0.05f, NULL, 0, 0.0f, 0.0f};
    kalman1d_t kf = {0.01f, 0.1f, 0.0f, 1.0f, 0.0f};

    // Erase flash before writing
    flash_erase();

    // Store PID and Kalman variables
    if (flash_write(PID_FLASH_OFFSET, &pid, sizeof(pid)) == 0)
        printf("PID saved to flash\n");
    if (flash_write(KALMAN_FLASH_OFFSET, &kf, sizeof(kf)) == 0)
        printf("Kalman saved to flash\n");

    // Load back and print
    pid_t pid2;
    kalman1d_t kf2;
    if (flash_read(PID_FLASH_OFFSET, &pid2, sizeof(pid2)) == 0)
        printf("PID loaded: kp=%.2f ki=%.2f kd=%.2f\n", pid2.kp, pid2.ki, pid2.kd);
    if (flash_read(KALMAN_FLASH_OFFSET, &kf2, sizeof(kf2)) == 0)
        printf("Kalman loaded: q=%.3f r=%.3f x=%.3f\n", kf2.q, kf2.r, kf2.x);
    return 0;
}
