#ifndef GPS_UART_H
#define GPS_UART_H
#include <stddef.h>
void gps_uart_init(void);
int gps_uart_read_line(char *out, size_t maxlen);
int gps_uart_parse_gga(const char *nmea, float *lat, float *lon);
#endif
