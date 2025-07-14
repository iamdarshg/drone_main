// hamming.c - Hamming code encode/decode implementation for robust comms
#include "hamming.h"
#include <stdint.h>

// Example: (7,4) Hamming code for 4 data bits, 3 parity bits
// You can expand to (15,11) or more for real use

uint8_t hamming_encode(uint8_t data) {
    // Only lower 4 bits used
    uint8_t d0 = (data >> 0) & 1;
    uint8_t d1 = (data >> 1) & 1;
    uint8_t d2 = (data >> 2) & 1;
    uint8_t d3 = (data >> 3) & 1;
    uint8_t p0 = d0 ^ d1 ^ d3;
    uint8_t p1 = d0 ^ d2 ^ d3;
    uint8_t p2 = d1 ^ d2 ^ d3;
    return (p0 << 6) | (p1 << 5) | (d3 << 4) | (p2 << 3) | (d2 << 2) | (d1 << 1) | d0;
}

uint8_t hamming_decode(uint8_t code, uint8_t *error) {
    uint8_t p0 = (code >> 6) & 1;
    uint8_t p1 = (code >> 5) & 1;
    uint8_t d3 = (code >> 4) & 1;
    uint8_t p2 = (code >> 3) & 1;
    uint8_t d2 = (code >> 2) & 1;
    uint8_t d1 = (code >> 1) & 1;
    uint8_t d0 = (code >> 0) & 1;
    uint8_t s0 = p0 ^ d0 ^ d1 ^ d3;
    uint8_t s1 = p1 ^ d0 ^ d2 ^ d3;
    uint8_t s2 = p2 ^ d1 ^ d2 ^ d3;
    uint8_t syndrome = (s2 << 2) | (s1 << 1) | s0;
    *error = 0;
    if (syndrome) {
        code ^= (1 << (7 - syndrome)); // Correct single-bit error
        *error = 1;
    }
    // Extract data bits
    return (d3 << 3) | (d2 << 2) | (d1 << 1) | d0;
}
