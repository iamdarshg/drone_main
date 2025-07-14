/*
 * main.c - Drone main application entry point (stub)
 *
 * Features:
 *  - Initializes hardware, RTOS, and all modules
 *  - Starts main control loop
 *  - Handles CLI and comms
 */
#include "board_config.h"
#include "pin_config.h"
#include "imu_lsm6ds3.h"
#include "imu_kx122.h"
#include "imu_lsm303c.h"
#include "rf_s2lpqtr.h"
#include "gps_uart.h"
#include "ext_uart.h"
#include "spi_bus.h"
#include "i2c_bus.h"
#include "flash.h"
#include "usb_bridge.h"
#include "hamming.h"
#include "kalman.h"
#include "pid.h"
#include "bus_monitor.h"
#include "core_monitor.h"
#include "error_handling.h"
#include "path_tracing.h"
#include "comms.h"
#include "frontend_cli.h"
#include "mission.h"

#include "logging.h"
#include "cli.h"
#include "init.h"

#include "Config/config.h"

#include "FreeRTOS.h"
#include "task.h"


static void MainTask(void *pvParameters) {
    (void)pvParameters;
    drivers_init();
    middleware_init();
    // --- Load PID/Kalman config from flash ---
    config_params_t cfg;
    if (config_load(&cfg) != 0) {
        log_info("Loaded default PID/Kalman config");
    } else {
        log_info("Loaded PID/Kalman config from flash");
    }
    // Example: initialize main PID and Kalman with config values
    // (Replace with your actual PID/Kalman instances and usage)
    float sensors[3] = {0};
    pid_t main_pid;
    pid_init(&main_pid, cfg.pid_kp, cfg.pid_ki, cfg.pid_kd, sensors, 3);
    kalman1d_t main_kf;
    kalman1d_init(&main_kf, cfg.kalman_q, cfg.kalman_r, 0.0f, 1.0f);
    // --- End config usage ---
    app_init();
    while (1) {
        app_main_loop();
    }
}

int main(void) {
    board_init();
    pin_init();
    log_info("Starting FreeRTOS...");
    xTaskCreate(MainTask, "MainTask", 1024, NULL, 1, NULL);
    vTaskStartScheduler();
    while (1) {}
    return 0;
}
