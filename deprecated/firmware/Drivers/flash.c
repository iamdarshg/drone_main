// flash.c - Simple Flash storage API for PID/Kalman variables
#include "flash.h"
#include <string.h>
#include <stdint.h>

// Example flash storage region (replace with real hardware address)
#define FLASH_STORAGE_BASE 0x08080000
#define FLASH_PAGE_SIZE   256

// QSPI Flash memory access (production, no simulation)
#include "spi_bus.h"

#define FLASH_CMD_WRITE_ENABLE  0x06
#define FLASH_CMD_PAGE_PROGRAM  0x02
#define FLASH_CMD_READ_DATA     0x03
#define FLASH_CMD_SECTOR_ERASE  0x20
#define FLASH_CMD_READ_STATUS   0x05

static int flash_wait_ready(void) {
    uint8_t cmd = FLASH_CMD_READ_STATUS;
    uint8_t status = 0x01; // Assume busy
    int timeout = 100000;
    while (status & 0x01) {
        if (spi_bus_write(&cmd, 1) < 0) return -1;
        if (spi_bus_read(&status, 1) < 0) return -1;
        if (--timeout == 0) return -2;
    }
    return 0;
}

int flash_write(uint32_t offset, const void *data, size_t len) {
    if (offset + len > FLASH_PAGE_SIZE) return -1;
    // Write enable
    uint8_t we = FLASH_CMD_WRITE_ENABLE;
    if (spi_bus_write(&we, 1) < 0) return -2;
    // Page program
    uint8_t cmd[4] = {FLASH_CMD_PAGE_PROGRAM,
                      (uint8_t)(offset >> 16),
                      (uint8_t)(offset >> 8),
                      (uint8_t)(offset)};
    if (spi_bus_write(cmd, 4) < 0) return -3;
    if (spi_bus_write((const uint8_t *)data, len) < 0) return -4;
    if (flash_wait_ready() < 0) return -5;
    return 0;
}

int flash_read(uint32_t offset, void *data, size_t len) {
    if (offset + len > FLASH_PAGE_SIZE) return -1;
    uint8_t cmd[4] = {FLASH_CMD_READ_DATA,
                      (uint8_t)(offset >> 16),
                      (uint8_t)(offset >> 8),
                      (uint8_t)(offset)};
    if (spi_bus_write(cmd, 4) < 0) return -2;
    if (spi_bus_read((uint8_t *)data, len) < 0) return -3;
    return 0;
}

int flash_erase(void) {
    // Write enable
    uint8_t we = FLASH_CMD_WRITE_ENABLE;
    if (spi_bus_write(&we, 1) < 0) return -1;
    // Sector erase (erase first sector only for now)
    uint8_t cmd[4] = {FLASH_CMD_SECTOR_ERASE, 0, 0, 0};
    if (spi_bus_write(cmd, 4) < 0) return -2;
    if (flash_wait_ready() < 0) return -3;
    return 0;
}
