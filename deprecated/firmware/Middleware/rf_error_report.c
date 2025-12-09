// rf_error_report.c - RF subsystem failure and error reporting
#include "rf_error_report.h"
#include "rf_monitor.h"
#include <stdio.h>

void rf_report_status(void) {
    printf("RF TX: %lu\n", (unsigned long)rf_monitor_get_tx());
    printf("RF RX: %lu\n", (unsigned long)rf_monitor_get_rx());
    printf("RF Errors: %lu\n", (unsigned long)rf_monitor_get_errors());
    printf("RF Signal Strength: %u\n", rf_monitor_get_signal());
    if (rf_monitor_get_errors() > 0) {
        printf("RF subsystem error detected!\n");
    }
}
