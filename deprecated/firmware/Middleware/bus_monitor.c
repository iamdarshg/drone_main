// bus_monitor.c - SPI/I2C bus usage monitoring
#include "bus_monitor.h"
#include "spi_bus.h"
#include "i2c_bus.h"
#include <stdint.h>

static volatile uint32_t spi_bytes = 0;
static volatile uint32_t i2c_bytes = 0;

void bus_monitor_spi_count(uint32_t n) { spi_bytes += n; }
void bus_monitor_i2c_count(uint32_t n) { i2c_bytes += n; }

uint32_t bus_monitor_get_spi_bytes(void) { return spi_bytes; }
uint32_t bus_monitor_get_i2c_bytes(void) { return i2c_bytes; }
void bus_monitor_reset(void) { spi_bytes = 0; i2c_bytes = 0; }
