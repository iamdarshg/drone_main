/*
 * interrupts.c - Drop-in interrupt handlers for firmware/Core/interrupts.c
 * LPC4330 interrupt configuration and handlers
 */

#ifdef HOST_BUILD
#include <stdio.h>
#include <signal.h>

void interrupts_init(void) {
    printf("Interrupts: Initializing (HOST BUILD)\n");
}

void interrupts_enable(void) {
    printf("Interrupts: Enabled (HOST BUILD)\n");
}

void interrupts_disable(void) {
    printf("Interrupts: Disabled (HOST BUILD)\n");
}

#else
// Real LPC4330 interrupt handling

#include <stdint.h>

// NVIC registers
#define NVIC_BASE           0xE000E100
#define NVIC_ISER_BASE      (NVIC_BASE + 0x000)
#define NVIC_ICER_BASE      (NVIC_BASE + 0x080)
#define NVIC_ISPR_BASE      (NVIC_BASE + 0x100)
#define NVIC_ICPR_BASE      (NVIC_BASE + 0x180)
#define NVIC_IPR_BASE       (NVIC_BASE + 0x300)

// LPC4330 IRQ numbers
typedef enum {
    DAC_IRQn              = 0,
    M0CORE_IRQn           = 1,
    DMA_IRQn              = 2,
    ETHERNET_IRQn         = 5,
    SDIO_IRQn             = 6,
    LCD_IRQn              = 7,
    USB0_IRQn             = 8,
    USB1_IRQn             = 9,
    SCT_IRQn              = 10,
    RIT_IRQn              = 11,
    TIMER0_IRQn           = 12,
    TIMER1_IRQn           = 13,
    TIMER2_IRQn           = 14,
    TIMER3_IRQn           = 15,
    MCPWM_IRQn            = 16,
    ADC0_IRQn             = 17,
    I2C0_IRQn             = 18,
    I2C1_IRQn             = 19,
    SPI_IRQn              = 20,
    ADC1_IRQn             = 21,
    SSP0_IRQn             = 22,
    SSP1_IRQn             = 23,
    UART0_IRQn            = 24,
    UART1_IRQn            = 25,
    UART2_IRQn            = 26,
    UART3_IRQn            = 27,
    I2S0_IRQn             = 28,
    I2S1_IRQn             = 29,
    SPIFI_IRQn            = 30,
    SGPIO_IRQn            = 31,
    GPIO0_IRQn            = 32,
    GPIO1_IRQn            = 33,
    GPIO2_IRQn            = 34,
    GPIO3_IRQn            = 35,
    GPIO4_IRQn            = 36,
    GPIO5_IRQn            = 37,
    GPIO6_IRQn            = 38,
    GPIO7_IRQn            = 39
} IRQn_Type;

void interrupts_init(void) {
    // Disable all interrupts initially
    for (int i = 0; i < 8; i++) {
        volatile uint32_t *icer = (volatile uint32_t *)(NVIC_ICER_BASE + i * 4);
        *icer = 0xFFFFFFFF;
    }
    
    // Clear all pending interrupts
    for (int i = 0; i < 8; i++) {
        volatile uint32_t *icpr = (volatile uint32_t *)(NVIC_ICPR_BASE + i * 4);
        *icpr = 0xFFFFFFFF;
    }
    
    // Set interrupt priorities (higher number = lower priority)
    nvic_set_priority(M0CORE_IRQn, 1);      // High priority for M0 communication
    nvic_set_priority(DMA_IRQn, 2);         // High priority for DMA
    nvic_set_priority(TIMER0_IRQn, 3);      // Medium priority for system timer
    nvic_set_priority(UART0_IRQn, 4);       // Medium priority for debug UART
    nvic_set_priority(I2C0_IRQn, 5);        // Lower priority for I2C
    nvic_set_priority(SPI_IRQn, 5);         // Lower priority for SPI
    nvic_set_priority(GPIO0_IRQn, 6);       // Lower priority for GPIO
    
    // Enable specific interrupts
    nvic_enable_irq(M0CORE_IRQn);
    nvic_enable_irq(TIMER0_IRQn);
    nvic_enable_irq(UART0_IRQn);
}

void interrupts_enable(void) {
    __enable_irq();
}

void interrupts_disable(void) {
    __disable_irq();
}

void nvic_enable_irq(IRQn_Type irq) {
    if (irq < 256) {
        volatile uint32_t *iser = (volatile uint32_t *)(NVIC_ISER_BASE + (irq / 32) * 4);
        *iser = (1 << (irq % 32));
    }
}

void nvic_disable_irq(IRQn_Type irq) {
    if (irq < 256) {
        volatile uint32_t *icer = (volatile uint32_t *)(NVIC_ICER_BASE + (irq / 32) * 4);
        *icer = (1 << (irq % 32));
    }
}

void nvic_set_priority(IRQn_Type irq, uint8_t priority) {
    if (irq < 256) {
        volatile uint8_t *ipr = (volatile uint8_t *)(NVIC_IPR_BASE + irq);
        *ipr = (priority << 4) & 0xFF;
    }
}

uint8_t nvic_get_priority(IRQn_Type irq) {
    if (irq < 256) {
        volatile uint8_t *ipr = (volatile uint8_t *)(NVIC_IPR_BASE + irq);
        return (*ipr >> 4) & 0x0F;
    }
    return 0;
}

void nvic_set_pending(IRQn_Type irq) {
    if (irq < 256) {
        volatile uint32_t *ispr = (volatile uint32_t *)(NVIC_ISPR_BASE + (irq / 32) * 4);
        *ispr = (1 << (irq % 32));
    }
}

void nvic_clear_pending(IRQn_Type irq) {
    if (irq < 256) {
        volatile uint32_t *icpr = (volatile uint32_t *)(NVIC_ICPR_BASE + (irq / 32) * 4);
        *icpr = (1 << (irq % 32));
    }
}

// Critical section helpers
static volatile uint32_t interrupt_nesting = 0;
static uint32_t saved_primask = 0;

void enter_critical_section(void) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    
    if (interrupt_nesting == 0) {
        saved_primask = primask;
    }
    interrupt_nesting++;
}

void exit_critical_section(void) {
    if (interrupt_nesting > 0) {
        interrupt_nesting--;
        if (interrupt_nesting == 0) {
            __set_PRIMASK(saved_primask);
        }
    }
}

// System timer configuration (1ms tick)
void systick_init(uint32_t core_clock_hz) {
    uint32_t ticks = core_clock_hz / 1000;  // 1ms interval
    
    // SysTick registers
    volatile uint32_t *SYST_CSR = (volatile uint32_t *)0xE000E010;
    volatile uint32_t *SYST_RVR = (volatile uint32_t *)0xE000E014;
    volatile uint32_t *SYST_CVR = (volatile uint32_t *)0xE000E018;
    
    *SYST_RVR = ticks - 1;
    *SYST_CVR = 0;
    *SYST_CSR = 0x07;  // Enable, interrupt, use core clock
}

// Timer interrupt handler
void TIMER0_IRQHandler(void) {
    // Clear timer interrupt
    extern void timer0_clear_interrupt(void);
    timer0_clear_interrupt();
    
    // System timer tick
    extern void system_timer_tick(void);
    system_timer_tick();
}

// UART interrupt handler for debug output
void UART0_IRQHandler(void) {
    extern void uart0_interrupt_handler(void);
    uart0_interrupt_handler();
}

// I2C interrupt handlers
void I2C0_IRQHandler(void) {
    extern void i2c0_interrupt_handler(void);
    i2c0_interrupt_handler();
}

void I2C1_IRQHandler(void) {
    extern void i2c1_interrupt_handler(void);
    i2c1_interrupt_handler();
}

// SPI interrupt handler
void SPI_IRQHandler(void) {
    extern void spi_interrupt_handler(void);
    spi_interrupt_handler();
}

// GPIO interrupt handlers
void GPIO0_IRQHandler(void) {
    extern void gpio0_interrupt_handler(void);
    gpio0_interrupt_handler();
}

// DMA interrupt handler
void DMA_IRQHandler(void) {
    extern void dma_interrupt_handler(void);
    dma_interrupt_handler();
}

#endif // HOST_BUILD
