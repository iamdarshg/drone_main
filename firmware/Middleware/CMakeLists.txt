# Middleware module for algorithms and monitoring
# (Stub for now)

add_library(Middleware STATIC
    hamming.c
    kalman.c
    pid.c
    bus_monitor.c
    core_monitor.c
    error_handling.c
    attitude_estimator.c
    telemetry.c
    command_processor.c
    safety_monitor.c
    safety_system.c
)

# TODO: Implement algorithms and monitoring
