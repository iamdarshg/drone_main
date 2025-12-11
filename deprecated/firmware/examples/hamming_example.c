// Example: Using Hamming code for error detection/correction
#include "hamming.h"
#include <stdio.h>

int main(void) {
    uint8_t data = 0x5A;
    uint16_t encoded = hamming_encode(data);
    printf("Original: 0x%02X, Encoded: 0x%04X\n", data, encoded);
    // Simulate a 1-bit error
    encoded ^= 0x0004;
    uint8_t decoded = hamming_decode(encoded);
    printf("Decoded: 0x%02X\n", decoded);
    return 0;
}
