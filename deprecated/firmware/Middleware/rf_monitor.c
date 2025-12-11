// rf_monitor.c - RF signal strength, usage, and error rate monitoring
#include "rf_monitor.h"
#include "rf_s2lpqtr.h"
#include <stdint.h>

static volatile uint32_t rf_tx_count = 0;
static volatile uint32_t rf_rx_count = 0;
static volatile uint32_t rf_error_count = 0;
static volatile uint8_t rf_signal_strength = 0;

void rf_monitor_tx_inc(void) { rf_tx_count++; }
void rf_monitor_rx_inc(void) { rf_rx_count++; }
void rf_monitor_error_inc(void) { rf_error_count++; }
void rf_monitor_set_signal(uint8_t strength) { rf_signal_strength = strength; }

uint32_t rf_monitor_get_tx(void) { return rf_tx_count; }
uint32_t rf_monitor_get_rx(void) { return rf_rx_count; }
uint32_t rf_monitor_get_errors(void) { return rf_error_count; }
uint8_t rf_monitor_get_signal(void) { return rf_signal_strength; }
void rf_monitor_reset(void) { rf_tx_count = rf_rx_count = rf_error_count = 0; rf_signal_strength = 0; }
