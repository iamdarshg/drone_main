#include "host_peripherals.h"
#include <string.h>

int spi_bus_write_host(const uint8_t *data, uint32_t len)
{
    // For host test, perform a simple transfer where rx is discarded
    uint8_t *rx = (uint8_t*)malloc(len);
    if (!rx) return -1;
    int ret = host_spi_transfer(data, rx, len);
    free(rx);
    return ret;
}

int spi_bus_read_host(uint8_t *data, uint32_t len)
{
    // Send NOPs to clock data out
    uint8_t *tx = (uint8_t*)malloc(len);
    if (!tx) return -1;
    memset(tx, 0xFF, len);
    int ret = host_spi_transfer(tx, data, len);
    free(tx);
    return ret;
}
