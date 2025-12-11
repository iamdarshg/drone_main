// gps_uart_example.c - Example: GPS over UART
#include "gps_uart.h"
#include <stdio.h>

int main(void) {
    gps_uart_init();
    char nmea[128];
    float lat = 0, lon = 0;
    if (gps_uart_read_line(nmea, sizeof(nmea)) > 0) {
        if (gps_uart_parse_gga(nmea, &lat, &lon) == 0) {
            printf("Lat: %f, Lon: %f\n", lat, lon);
        } else {
            printf("Failed to parse GGA\n");
        }
    } else {
        printf("No NMEA data\n");
    }
    return 0;
}
