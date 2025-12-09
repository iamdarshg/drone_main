// gps_uart.c - Full GPS over UART implementation
#include "gps_uart.h"
#include <stdint.h>
#include <string.h>
#include "ext_uart.h"

#ifndef GPS_UART_PORT
#define GPS_UART_PORT 2 // Now configurable via build system or config.h
#endif
#ifndef GPS_UART_BAUD
#define GPS_UART_BAUD 9600 // Now configurable via build system or config.h
#endif

static char gps_rx_buffer[128];

void gps_uart_init(void) {
    ext_uart_init(GPS_UART_PORT, GPS_UART_BAUD);
}

// Read a line (NMEA sentence) from GPS
int gps_uart_read_line(char *out, size_t maxlen) {
    size_t i = 0;
    char c = 0;
    while (i < maxlen - 1) {
        if (ext_uart_read(GPS_UART_PORT, (uint8_t *)&c, 1) != 1) break;
        if (c == '\n') break;
        out[i++] = c;
    }
    out[i] = '\0';
    return (int)i;
}

// Parse a simple NMEA GGA sentence for latitude/longitude
int gps_uart_parse_gga(const char *nmea, float *lat, float *lon) {
    // Example: $GPGGA,123519,4807.038,N,01131.000,E,...
    const char *p = strstr(nmea, "$GPGGA");
    if (!p) return -1;
    int hh, mm, ss;
    float raw_lat, raw_lon;
    char ns, ew;
    if (sscanf(nmea, "$GPGGA,%2d%2d%2d,%f,%c,%f,%c", &hh, &mm, &ss, &raw_lat, &ns, &raw_lon, &ew) < 7) return -1;
    *lat = (int)(raw_lat / 100) + fmodf(raw_lat, 100) / 60.0f;
    if (ns == 'S') *lat = -*lat;
    *lon = (int)(raw_lon / 100) + fmodf(raw_lon, 100) / 60.0f;
    if (ew == 'W') *lon = -*lon;
    return 0;
}
