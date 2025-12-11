#ifndef RF_MONITOR_H
#define RF_MONITOR_H
#include <stdint.h>
void rf_monitor_tx_inc(void);
void rf_monitor_rx_inc(void);
void rf_monitor_error_inc(void);
void rf_monitor_set_signal(uint8_t strength);
uint32_t rf_monitor_get_tx(void);
uint32_t rf_monitor_get_rx(void);
uint32_t rf_monitor_get_errors(void);
uint8_t rf_monitor_get_signal(void);
void rf_monitor_reset(void);
#endif
