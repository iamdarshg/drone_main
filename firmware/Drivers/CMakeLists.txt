# Drivers module for hardware abstraction
# (Stub for now)

add_library(Drivers STATIC
    imu_lsm6ds3.c
    imu_kx122.c
    imu_lsm303c.c
    rf_s2lpqtr.c
    gps_uart.c
    ext_uart.c
    spi_bus.c
    i2c_bus.c
    flash.c
    usb_bridge.c
)

# TODO: Implement each driver
