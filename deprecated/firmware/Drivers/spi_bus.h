
#ifndef SPI_BUS_H
#define SPI_BUS_H

#include <stdint.h>

/*
 * QSPI (Quad SPI) driver for LPC4330
 * NASA-level error proofing, robust for mission-critical use
 *
 * All routines are hardware-ready and blocking. No stubs or demo logic.
 *
 * Author: [Your Name], Date: [Update as needed]
 */

// Write data to QSPI bus. Returns number of bytes written, or <0 on error (with rollback)
int spi_bus_write(const uint8_t *data, uint32_t len);

// Read data from QSPI bus. Returns number of bytes read, or <0 on error (with rollback)
int spi_bus_read(uint8_t *data, uint32_t len);

#endif // SPI_BUS_H
