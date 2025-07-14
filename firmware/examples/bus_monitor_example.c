// bus_monitor_example.c - Example: Monitor SPI/I2C bus usage
#include "bus_monitor.h"
#include <stdio.h>

int main(void) {
    bus_monitor_reset();
    // Simulate some SPI/I2C transfers
    bus_monitor_spi_count(128);
    bus_monitor_i2c_count(64);
    printf("SPI bytes: %lu\n", (unsigned long)bus_monitor_get_spi_bytes());
    printf("I2C bytes: %lu\n", (unsigned long)bus_monitor_get_i2c_bytes());
    return 0;
}
