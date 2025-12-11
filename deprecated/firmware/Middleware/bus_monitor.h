#ifndef BUS_MONITOR_H
#define BUS_MONITOR_H
#include <stdint.h>
void bus_monitor_spi_count(uint32_t n);
void bus_monitor_i2c_count(uint32_t n);
uint32_t bus_monitor_get_spi_bytes(void);
uint32_t bus_monitor_get_i2c_bytes(void);
void bus_monitor_reset(void);
#endif
