#ifndef SPI_BUS_H
#define SPI_BUS_H
#include <stdint.h>

// Write data to SPI bus. Returns number of bytes written, or <0 on error (with rollback)
int spi_bus_write(const uint8_t *data, uint32_t len);
// Read data from SPI bus. Returns number of bytes read, or <0 on error (with rollback)
int spi_bus_read(uint8_t *data, uint32_t len);

#endif
