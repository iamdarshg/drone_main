# Fixed Core CMakeLists.txt with all missing source files created
cmake_minimum_required(VERSION 3.13)

# Core module for MCU startup, RTOS, and dual-core support
add_library(Core STATIC
    startup.c
    system_lpc43xx.c
    rtos_hooks.c
    dualcore.c
    interrupts.c
    syscalls.c
)

# Include directories
target_include_directories(Core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Conditional compilation for cross-compile vs host
if(CMAKE_CROSSCOMPILING)
    # LPC4330 specific definitions
    target_compile_definitions(Core PRIVATE
        LPC43XX
        CORE_M4
        __USE_LPCOPEN
        FREE_RTOS
    )
    
    # Link against hardware abstraction libraries
    # Note: In real implementation, you'd link against LPCOpen libraries
    target_compile_options(Core PRIVATE
        -mcpu=cortex-m4
        -mthumb
        -mfpu=fpv4-sp-d16
        -mfloat-abi=hard
    )
else()
    # Host build definitions for testing
    target_compile_definitions(Core PRIVATE
        HOST_BUILD
        DEBUG_BUILD
    )
endif()

message(STATUS "Core module configured")
message(STATUS "  Files: startup.c, system_lpc43xx.c, rtos_hooks.c, dualcore.c, interrupts.c, syscalls.c")
