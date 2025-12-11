// rf_monitor_example.c - Example: RF signal, usage, and error monitoring
#include "rf_monitor.h"
#include <stdio.h>

int main(void) {
    rf_monitor_reset();
    rf_monitor_tx_inc();
    rf_monitor_rx_inc();
    rf_monitor_error_inc();
    rf_monitor_set_signal(75);
    printf("RF TX: %lu\n", (unsigned long)rf_monitor_get_tx());
    printf("RF RX: %lu\n", (unsigned long)rf_monitor_get_rx());
    printf("RF Errors: %lu\n", (unsigned long)rf_monitor_get_errors());
    printf("RF Signal Strength: %u\n", rf_monitor_get_signal());
    return 0;
}
