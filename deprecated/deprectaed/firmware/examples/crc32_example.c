// Example: Using CRC32 for data integrity
#include "crc32.h"
#include <stdio.h>

int main(void) {
    const char *data = "Hello, CRC32!";
    uint32_t crc = crc32_compute((const uint8_t*)data, strlen(data));
    printf("CRC32: 0x%08X\n", crc);
    return 0;
}
