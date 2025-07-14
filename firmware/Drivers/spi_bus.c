#include "spi_bus.h"
#include "bus_monitor.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

// Hardware-specific: Chip select/deselect (replace with actual hardware calls)
static void spi_bus_select(void)   { /* Set CS low */ }
static void spi_bus_deselect(void) { /* Set CS high */ }
static void spi_bus_reset(void)    { /* Reset SPI peripheral if needed */ }

// Example: Hardware SPI write (replace with actual hardware call)
static int hw_spi_write(const uint8_t *data, uint32_t len) {
    // Return number of bytes written, or <0 on error
    // ...hardware-specific code...
    return len; // Stub: always succeeds
}

// Example: Hardware SPI read (replace with actual hardware call)
static int hw_spi_read(uint8_t *data, uint32_t len) {
    // Return number of bytes read, or <0 on error
    // ...hardware-specific code...
    return len; // Stub: always succeeds
}

// Rollback handler: robust error recovery
static void spi_bus_rollback(const char *src) {
    // Deselect chip, reset bus, log error
    spi_bus_deselect();
    spi_bus_reset();
    bus_monitor_reset();
    if (src) printf("[SPI_BUS] Error, rollback from: %s\n", src);
}

int spi_bus_write(const uint8_t *data, uint32_t len) {
    if (!data || len == 0) return -1;
    spi_bus_select();
    int ret = hw_spi_write(data, len);
    if (ret < 0) {
        spi_bus_rollback("spi_bus_write");
        return -1;
    }
    spi_bus_deselect();
    bus_monitor_spi_count(len);
    return ret;
}

int spi_bus_read(uint8_t *data, uint32_t len) {
    if (!data || len == 0) return -1;
    spi_bus_select();
    int ret = hw_spi_read(data, len);
    if (ret < 0) {
        spi_bus_rollback("spi_bus_read");
        return -1;
    }
    spi_bus_deselect();
    bus_monitor_spi_count(len);
    return ret;
}