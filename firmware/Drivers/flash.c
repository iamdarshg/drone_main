// flash.c - Simple Flash storage API for PID/Kalman variables
#include "flash.h"
#include <string.h>
#include <stdint.h>

// Example flash storage region (replace with real hardware address)
#define FLASH_STORAGE_BASE 0x08080000
#define FLASH_PAGE_SIZE   256

// Simulated flash memory (for test/demo)
static uint8_t flash_mem[FLASH_PAGE_SIZE];

int flash_write(uint32_t offset, const void *data, size_t len) {
    if (offset + len > FLASH_PAGE_SIZE) return -1;
    memcpy(&flash_mem[offset], data, len);
    return 0;
}

int flash_read(uint32_t offset, void *data, size_t len) {
    if (offset + len > FLASH_PAGE_SIZE) return -1;
    memcpy(data, &flash_mem[offset], len);
    return 0;
}

int flash_erase(void) {
    memset(flash_mem, 0xFF, FLASH_PAGE_SIZE);
    return 0;
}
