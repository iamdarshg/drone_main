/*
 * rtos_hooks.c - Drop-in FreeRTOS hooks for firmware/Core/rtos_hooks.c
 * FreeRTOS callback functions and memory management
 */

#ifdef HOST_BUILD
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <time.h>
#include <unistd.h>

// Host simulation of FreeRTOS
static int rtos_running = 0;

void vApplicationStackOverflowHook(void *xTask, signed char *pcTaskName) {
    printf("RTOS: Stack overflow in task: %s\n", pcTaskName);
    exit(1);
}

void vApplicationMallocFailedHook(void) {
    printf("RTOS: Malloc failed!\n");
    exit(1);
}

void vApplicationIdleHook(void) {
    // Simulate idle task
    usleep(1000); // 1ms delay
}

void vApplicationTickHook(void) {
    // Simulate system tick
    static int tick_count = 0;
    tick_count++;
    if (tick_count % 1000 == 0) {
        printf("RTOS: Tick %d (HOST BUILD)\n", tick_count);
    }
}

void vApplicationGetIdleTaskMemory(void **ppxIdleTaskTCBBuffer, 
                                  void **ppxIdleTaskStackBuffer, 
                                  uint32_t *pulIdleTaskStackSize) {
    static void *idle_tcb;
    static void *idle_stack;
    
    idle_tcb = malloc(256);     // Simulate TCB
    idle_stack = malloc(1024);  // Simulate stack
    
    *ppxIdleTaskTCBBuffer = idle_tcb;
    *ppxIdleTaskStackBuffer = idle_stack;
    *pulIdleTaskStackSize = 1024;
}

// Host simulation task scheduler
void vTaskStartScheduler(void) {
    printf("RTOS: Starting scheduler (HOST BUILD)\n");
    rtos_running = 1;
    
    // Simulate running forever
    while (rtos_running) {
        vApplicationIdleHook();
        vApplicationTickHook();
    }
}

void vTaskEndScheduler(void) {
    printf("RTOS: Stopping scheduler\n");
    rtos_running = 0;
}

#else
// Real FreeRTOS hooks for LPC4330

#include <stdint.h>
#include <stdlib.h>

// Static memory allocation for FreeRTOS
#define configTOTAL_HEAP_SIZE    (64 * 1024)  // 64KB heap

// Heap memory pool
static uint8_t ucHeap[configTOTAL_HEAP_SIZE];
static size_t xNextFreeByte = 0;

// Stack overflow hook
void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName) {
    (void)xTask;
    (void)pcTaskName;
    
    // Log the error
    // In a real system, you might save crash data to flash
    
    // Infinite loop to stop execution
    __disable_irq();
    while (1) {
        // Trap here - indicates stack overflow
    }
}

// Malloc failed hook
void vApplicationMallocFailedHook(void) {
    // Log the error
    
    // Infinite loop to stop execution
    __disable_irq();
    while (1) {
        // Trap here - indicates out of memory
    }
}

// Idle hook - called when system is idle
void vApplicationIdleHook(void) {
    // Put CPU to sleep to save power
    __asm__("wfi");  // Wait for interrupt
    
    // Could also do:
    // - Watchdog reset
    // - Background tasks
    // - Power management
}

// Tick hook - called every system tick (1ms typically)
void vApplicationTickHook(void) {
    // Update system time
    static uint32_t tick_count = 0;
    tick_count++;
    
    // Could be used for:
    // - Periodic tasks
    // - Watchdog kicking
    // - System monitoring
    
    // Kick watchdog every 100ms
    if (tick_count % 100 == 0) {
        extern void watchdog_kick(void);
        watchdog_kick();
    }
}

// Get idle task memory - for static allocation
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                  StackType_t **ppxIdleTaskStackBuffer,
                                  uint32_t *pulIdleTaskStackSize) {
    static StaticTask_t xIdleTaskTCB;
    static StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE];
    
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

// Get timer task memory - for static allocation
void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                   StackType_t **ppxTimerTaskStackBuffer,
                                   uint32_t *pulTimerTaskStackSize) {
    static StaticTask_t xTimerTaskTCB;
    static StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH];
    
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

// Simple heap implementation
void* pvPortMalloc(size_t xWantedSize) {
    void *pvReturn = NULL;
    
    // Align to 8-byte boundary
    xWantedSize = (xWantedSize + 7) & ~7;
    
    if ((xNextFreeByte + xWantedSize) < configTOTAL_HEAP_SIZE) {
        pvReturn = &ucHeap[xNextFreeByte];
        xNextFreeByte += xWantedSize;
    }
    
    return pvReturn;
}

void vPortFree(void* pv) {
    // Simple heap - no free implementation
    // In production, use heap_4.c or heap_5.c from FreeRTOS
    (void)pv;
}

size_t xPortGetFreeHeapSize(void) {
    return configTOTAL_HEAP_SIZE - xNextFreeByte;
}

// Critical section handling
void vPortEnterCritical(void) {
    __disable_irq();
}

void vPortExitCritical(void) {
    __enable_irq();
}

// Assert hook
void vAssertCalled(const char *pcFile, uint32_t ulLine) {
    (void)pcFile;
    (void)ulLine;
    
    // Log assert information
    
    // Stop execution
    __disable_irq();
    while (1) {
        // Trap here - indicates assertion failure
    }
}

// Daemon task startup hook
void vApplicationDaemonTaskStartupHook(void) {
    // Called when the RTOS daemon task starts
    // Can be used to create other tasks
    
    // Create safety monitoring task
    extern void create_system_tasks(void);
    create_system_tasks();
}

// Pre-sleep processing
void vApplicationSleep(uint32_t ulExpectedIdleTime) {
    (void)ulExpectedIdleTime;
    
    // Prepare for sleep mode
    // Could implement:
    // - Clock gating
    // - Peripheral power down
    // - Deep sleep modes
    
    __asm__("wfi");
}

// Watchdog kick function
void watchdog_kick(void) {
    // Kick hardware watchdog
    // This would normally write to watchdog registers
    
    // Also kick M0 watchdog via inter-core communication
    extern void m0_watchdog_kick(void);
    m0_watchdog_kick();
}

#endif // HOST_BUILD