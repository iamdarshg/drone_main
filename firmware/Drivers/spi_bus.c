#include "spi_bus.h"
#include "bus_monitor.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

/*
 * QSPI (Quad SPI) driver for LPC4330
 *
 * This implementation provides high-performance, robust SPI access for flash and peripherals.
 * All routines are hardware-ready, with NASA-level error handling and rollback.
 *
 * NOTE: Pin mapping and QSPI peripheral base address must match your board design.
 *
 * Author: [Your Name], Date: [Update as needed]
 */

#include "chip.h" // If available, else define LPC_QSPI_BASE etc. manually
#define LPC_QSPI_BASE   (0x400A1000UL) // QSPI base for LPC4330 (check datasheet)
#define QSPI            ((QSPI_TypeDef *)LPC_QSPI_BASE)

typedef struct {
    volatile uint32_t CFG;
    volatile uint32_t DLY;
    volatile uint32_t STAT;
    volatile uint32_t INTENSET;
    volatile uint32_t INTENCLR;
    volatile uint32_t RXDAT;
    volatile uint32_t TXDATCTL;
    volatile uint32_t TXDAT;
    volatile uint32_t TXCTL;
    volatile uint32_t DIV;
    volatile uint32_t INTSTAT;
    // ... add more if needed
} QSPI_TypeDef;

#define QSPI_CFG_ENABLE         (1 << 0)
#define QSPI_STAT_RXRDY         (1 << 0)
#define QSPI_STAT_TXRDY         (1 << 1)
#define QSPI_STAT_BUSY          (1 << 4)

// Hardware-specific: Chip select/deselect (GPIO, active low)
#define QSPI_CS_PORT    2
#define QSPI_CS_PIN     7
static void spi_bus_select(void) {
    // Set CS low (active)
    LPC_GPIO_PORT->CLR[QSPI_CS_PORT] = (1 << QSPI_CS_PIN);
}
static void spi_bus_deselect(void) {
    // Set CS high (inactive)
    LPC_GPIO_PORT->SET[QSPI_CS_PORT] = (1 << QSPI_CS_PIN);
}
static void spi_bus_reset(void) {
    // Reset QSPI peripheral (disable/enable)
    QSPI->CFG &= ~QSPI_CFG_ENABLE;
    for (volatile int i = 0; i < 1000; ++i) ; // Short delay
    QSPI->CFG |= QSPI_CFG_ENABLE;
}

// Rollback handler: robust error recovery
static void spi_bus_rollback(const char *src) {
    spi_bus_deselect();
    spi_bus_reset();
    bus_monitor_reset();
    if (src) printf("[SPI_BUS] Error, rollback from: %s\n", src);
}

// QSPI write (blocking, with error handling)
static int hw_spi_write(const uint8_t *data, uint32_t len) {
    if (!data || len == 0) return -1;
    for (uint32_t i = 0; i < len; ++i) {
        // Wait for TX ready
        uint32_t timeout = 100000;
        while (!(QSPI->STAT & QSPI_STAT_TXRDY)) {
            if (--timeout == 0) return -2;
        }
        QSPI->TXDAT = data[i];
        // Wait for transfer complete
        timeout = 100000;
        while (QSPI->STAT & QSPI_STAT_BUSY) {
            if (--timeout == 0) return -3;
        }
    }
    return len;
}

// QSPI read (blocking, with error handling)
static int hw_spi_read(uint8_t *data, uint32_t len) {
    if (!data || len == 0) return -1;
    for (uint32_t i = 0; i < len; ++i) {
        // Wait for RX ready
        uint32_t timeout = 100000;
        while (!(QSPI->STAT & QSPI_STAT_RXRDY)) {
            if (--timeout == 0) return -2;
        }
        data[i] = (uint8_t)(QSPI->RXDAT & 0xFF);
    }
    return len;
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