/*
 * startup.c - Drop-in startup code for firmware/Core/startup.c
 * LPC4330 M4 core startup with dual-core initialization
 */

#ifdef HOST_BUILD
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>

// Host build stub
int main(int argc, char *argv[]);

void startup_init(void) {
    printf("Drone firmware starting (HOST BUILD)\n");
}

// Host signal handler for clean shutdown
void host_signal_handler(int sig) {
    printf("Received signal %d, shutting down...\n", sig);
    exit(0);
}

// Host main function
int main(int argc, char *argv[]) {
    (void)argc;
    (void)argv;
    
    printf("=== Drone Firmware Host Test Build ===\n");
    printf("This is a simulation build for testing on host\n");
    
    // Register signal handlers
    signal(SIGINT, host_signal_handler);
    signal(SIGTERM, host_signal_handler);
    
    startup_init();
    
    // Call the main application entry point
    extern void application_main(void);
    application_main();
    
    return 0;
}

#else
// Real LPC4330 startup code

#include <stdint.h>

// LPC4330 Vector table and startup
extern uint32_t _estack;
extern uint32_t _sidata;
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _sbss;
extern uint32_t _ebss;

// Application main entry point
extern void application_main(void);

// Reset handler
void Reset_Handler(void) {
    // Copy initialized data from flash to RAM
    uint32_t *src = &_sidata;
    uint32_t *dst = &_sdata;
    
    while (dst < &_edata) {
        *dst++ = *src++;
    }
    
    // Zero out BSS section
    dst = &_sbss;
    while (dst < &_ebss) {
        *dst++ = 0;
    }
    
    // Initialize system
    startup_init();
    
    // Call main application
    application_main();
    
    // Should never return
    while (1) {
        __asm__("wfi"); // Wait for interrupt
    }
}

// Default interrupt handler
void Default_Handler(void) {
    while (1) {
        // Trap in infinite loop
    }
}

// Core interrupts for Cortex-M4
void NMI_Handler(void) __attribute__((weak, alias("Default_Handler")));
void HardFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
void MemManage_Handler(void) __attribute__((weak, alias("Default_Handler")));
void BusFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
void UsageFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SVC_Handler(void) __attribute__((weak, alias("Default_Handler")));
void DebugMon_Handler(void) __attribute__((weak, alias("Default_Handler")));
void PendSV_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SysTick_Handler(void) __attribute__((weak, alias("Default_Handler")));

// LPC4330 specific interrupts
void DAC_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void M0CORE_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DMA_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void ETHERNET_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void SDIO_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void LCD_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void USB0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void USB1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void SCT_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void RIT_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIMER0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIMER1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIMER2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIMER3_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void MCPWM_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void ADC0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void I2C0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void I2C1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void SPI_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void ADC1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void SSP0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void SSP1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void UART0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void UART1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void UART2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void UART3_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void I2S0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void I2S1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void SPIFI_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void SGPIO_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void GPIO0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void GPIO1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void GPIO2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void GPIO3_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void GPIO4_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void GPIO5_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void GPIO6_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void GPIO7_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));

// Vector table
__attribute__((section(".isr_vector"), used))
void (* const vector_table[])(void) = {
    (void (*)(void))&_estack,    // Initial Stack Pointer
    Reset_Handler,               // Reset Handler
    NMI_Handler,                 // NMI Handler
    HardFault_Handler,           // Hard Fault Handler
    MemManage_Handler,           // MPU Fault Handler
    BusFault_Handler,            // Bus Fault Handler
    UsageFault_Handler,          // Usage Fault Handler
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    SVC_Handler,                 // SVCall Handler
    DebugMon_Handler,            // Debug Monitor Handler
    0,                           // Reserved
    PendSV_Handler,              // PendSV Handler
    SysTick_Handler,             // SysTick Handler
    
    // LPC4330 specific interrupts
    DAC_IRQHandler,              // 16
    M0CORE_IRQHandler,           // 17
    DMA_IRQHandler,              // 18
    0,                           // 19 Reserved
    0,                           // 20 Reserved
    ETHERNET_IRQHandler,         // 21
    SDIO_IRQHandler,             // 22
    LCD_IRQHandler,              // 23
    USB0_IRQHandler,             // 24
    USB1_IRQHandler,             // 25
    SCT_IRQHandler,              // 26
    RIT_IRQHandler,              // 27
    TIMER0_IRQHandler,           // 28
    TIMER1_IRQHandler,           // 29
    TIMER2_IRQHandler,           // 30
    TIMER3_IRQHandler,           // 31
    MCPWM_IRQHandler,            // 32
    ADC0_IRQHandler,             // 33
    I2C0_IRQHandler,             // 34
    I2C1_IRQHandler,             // 35
    SPI_IRQHandler,              // 36
    ADC1_IRQHandler,             // 37
    SSP0_IRQHandler,             // 38
    SSP1_IRQHandler,             // 39
    UART0_IRQHandler,            // 40
    UART1_IRQHandler,            // 41
    UART2_IRQHandler,            // 42
    UART3_IRQHandler,            // 43
    I2S0_IRQHandler,             // 44
    I2S1_IRQHandler,             // 45
    SPIFI_IRQHandler,            // 46
    SGPIO_IRQHandler,            // 47
    GPIO0_IRQHandler,            // 48
    GPIO1_IRQHandler,            // 49
    GPIO2_IRQHandler,            // 50
    GPIO3_IRQHandler,            // 51
    GPIO4_IRQHandler,            // 52
    GPIO5_IRQHandler,            // 53
    GPIO6_IRQHandler,            // 54
    GPIO7_IRQHandler             // 55
};

void startup_init(void) {
    // Enable FPU
    #if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  // set CP10 and CP11 Full Access
    #endif
    
    // Configure system clock - would normally call SystemInit()
    // For now, just enable basic clocks
    
    // Initialize dual-core communication
    extern void dualcore_init(void);
    dualcore_init();
}

#endif // HOST_BUILD