# Core module for MCU startup, RTOS, and dual-core support
# (Stub for now)

add_library(Core STATIC
    startup.c
    system_lpc43xx.c
    rtos_hooks.c
    dualcore.c
)

# TODO: Add LPCOpen, FreeRTOS, and linker script integration
