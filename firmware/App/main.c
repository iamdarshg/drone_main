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

int main(void) {
    // Initialize board and peripherals
    board_init();
    pin_init();
    drivers_init();
    middleware_init();
    app_init();

    // Start RTOS scheduler (if used)
    // vTaskStartScheduler();

    // Main loop (if baremetal)
    while (1) {
        app_main_loop();
    }
    return 0;
}
