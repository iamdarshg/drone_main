#ifndef FLASH_H
#define FLASH_H
#include <stddef.h>
#include <stdint.h>

// Write data to flash at offset. Returns 0 on success, <0 on error.
int flash_write(uint32_t offset, const void *data, size_t len);
// Read data from flash at offset. Returns 0 on success, <0 on error.
int flash_read(uint32_t offset, void *data, size_t len);
// Erase flash storage region. Returns 0 on success, <0 on error.
int flash_erase(void);

#endif
