/*
 * system_lpc43xx.c - Drop-in system initialization for firmware/Core/system_lpc43xx.c
 * LPC4330 system configuration and clock setup
 */

#ifdef HOST_BUILD
#include <stdio.h>

uint32_t SystemCoreClock = 180000000;  // Simulated 180MHz

void SystemInit(void) {
    printf("SystemInit (HOST BUILD): Simulating LPC4330 initialization\n");
}

void SystemCoreClockUpdate(void) {
    printf("SystemCoreClockUpdate (HOST BUILD): Core clock = %lu Hz\n", SystemCoreClock);
}

#else
// Real LPC4330 system initialization

#include <stdint.h>

// System clock frequency
uint32_t SystemCoreClock = 12000000;  // Default to IRC (12MHz)

// LPC4330 register definitions (simplified)
#define LPC_CGU_BASE        0x40050000
#define LPC_CCU1_BASE       0x40051000  
#define LPC_CCU2_BASE       0x40052000
#define LPC_CREG_BASE       0x40043000

// CGU registers
typedef struct {
    volatile uint32_t FREQ_MON;
    volatile uint32_t XTAL_OSC_CTRL;
    volatile uint32_t PLL0USB_STAT;
    volatile uint32_t PLL0USB_CTRL;
    volatile uint32_t PLL0AUDIO_STAT;
    volatile uint32_t PLL0AUDIO_CTRL;
    volatile uint32_t PLL0AUDIO_MDIV;
    volatile uint32_t PLL0AUDIO_NP_DIV;
    volatile uint32_t PLL1_STAT;
    volatile uint32_t PLL1_CTRL;
    volatile uint32_t IDIVA_CTRL;
    volatile uint32_t IDIVB_CTRL;
    volatile uint32_t IDIVC_CTRL;
    volatile uint32_t IDIVD_CTRL;
    volatile uint32_t IDIVE_CTRL;
    volatile uint32_t BASE_SAFE_CLK;
    volatile uint32_t BASE_USB0_CLK;
    volatile uint32_t BASE_PERIPH_CLK;
    volatile uint32_t BASE_USB1_CLK;
    volatile uint32_t BASE_M4_CLK;
    volatile uint32_t BASE_SPIFI_CLK;
    volatile uint32_t BASE_SPI_CLK;
    volatile uint32_t BASE_PHY_RX_CLK;
    volatile uint32_t BASE_PHY_TX_CLK;
    volatile uint32_t BASE_APB1_CLK;
    volatile uint32_t BASE_APB3_CLK;
    volatile uint32_t BASE_LCD_CLK;
    volatile uint32_t BASE_VADC_CLK;
    volatile uint32_t BASE_SDIO_CLK;
    volatile uint32_t BASE_SSP0_CLK;
    volatile uint32_t BASE_SSP1_CLK;
    volatile uint32_t BASE_UART0_CLK;
    volatile uint32_t BASE_UART1_CLK;
    volatile uint32_t BASE_UART2_CLK;
    volatile uint32_t BASE_UART3_CLK;
    volatile uint32_t BASE_OUT_CLK;
    volatile uint32_t BASE_RES1[4];
    volatile uint32_t BASE_APLL_CLK;
    volatile uint32_t BASE_CGU_OUT0_CLK;
    volatile uint32_t BASE_CGU_OUT1_CLK;
} LPC_CGU_T;

#define LPC_CGU ((LPC_CGU_T *) LPC_CGU_BASE)

// Clock sources
#define CGU_SRC_32KHZ       0x00
#define CGU_SRC_IRC         0x01
#define CGU_SRC_ENET_RX     0x02
#define CGU_SRC_ENET_TX     0x03
#define CGU_SRC_GP_CLKIN    0x04
#define CGU_SRC_XTAL        0x06
#define CGU_SRC_PLL0USB     0x07
#define CGU_SRC_PLL0AUDIO   0x08
#define CGU_SRC_PLL1        0x09
#define CGU_SRC_IDIVA       0x0C
#define CGU_SRC_IDIVB       0x0D
#define CGU_SRC_IDIVC       0x0E
#define CGU_SRC_IDIVD       0x0F
#define CGU_SRC_IDIVE       0x10

// CGU control bits
#define CGU_CTRL_AUTOBLOCK  (1 << 11)
#define CGU_CTRL_CLK_SEL(x) ((x) & 0x1F)
#define CGU_CTRL_EN         (1 << 0)

// PLL control bits
#define CGU_PLL_CTRL_PD     (1 << 0)
#define CGU_PLL_CTRL_BYPASS (1 << 1)
#define CGU_PLL_CTRL_FBSEL  (1 << 6)
#define CGU_PLL_CTRL_DIRECT (1 << 7)
#define CGU_PLL_CTRL_PSEL(x) (((x) & 0x3) << 8)
#define CGU_PLL_CTRL_NSEL(x) (((x) & 0x3) << 12)
#define CGU_PLL_CTRL_MSEL(x) (((x) & 0xFF) << 16)

// CREG registers for M0 core control
typedef struct {
    volatile uint32_t reserved[64];
    volatile uint32_t M0SUBMEMMAP;
    volatile uint32_t reserved1[63];
    volatile uint32_t M0APPMEMMAP;
} LPC_CREG_T;

#define LPC_CREG ((LPC_CREG_T *) LPC_CREG_BASE)

void SystemInit(void) {
    // Enable crystal oscillator if using external crystal
    // For now, we'll use the internal RC oscillator (IRC) at 12MHz
    
    // Set up PLL1 for main system clock
    // PLL1 input from IRC (12MHz)
    // Target: 180MHz (12MHz * 15 = 180MHz)
    
    // Configure PLL1
    LPC_CGU->PLL1_CTRL = CGU_PLL_CTRL_PD;  // Power down first
    
    // Set PLL1 parameters: M=15, N=1, P=1 for 180MHz output
    LPC_CGU->PLL1_CTRL = CGU_PLL_CTRL_BYPASS | 
                         CGU_PLL_CTRL_FBSEL |
                         CGU_PLL_CTRL_MSEL(14) |   // M = 15 (14+1)
                         CGU_PLL_CTRL_NSEL(0) |    // N = 1 (0+1) 
                         CGU_PLL_CTRL_PSEL(0) |    // P = 1 (2^0)
                         CGU_SRC_IRC;              // Input from IRC
    
    // Remove power down
    LPC_CGU->PLL1_CTRL &= ~CGU_PLL_CTRL_PD;
    
    // Wait for PLL1 lock (simplified - normally check lock bit)
    for (volatile int i = 0; i < 10000; i++);
    
    // Remove bypass
    LPC_CGU->PLL1_CTRL &= ~CGU_PLL_CTRL_BYPASS;
    
    // Set base clocks
    LPC_CGU->BASE_M4_CLK = CGU_CTRL_AUTOBLOCK | CGU_CTRL_CLK_SEL(CGU_SRC_PLL1);
    LPC_CGU->BASE_APB1_CLK = CGU_CTRL_AUTOBLOCK | CGU_CTRL_CLK_SEL(CGU_SRC_PLL1);
    LPC_CGU->BASE_APB3_CLK = CGU_CTRL_AUTOBLOCK | CGU_CTRL_CLK_SEL(CGU_SRC_PLL1);
    
    // Set peripheral clocks
    LPC_CGU->BASE_UART0_CLK = CGU_CTRL_AUTOBLOCK | CGU_CTRL_CLK_SEL(CGU_SRC_PLL1);
    LPC_CGU->BASE_UART1_CLK = CGU_CTRL_AUTOBLOCK | CGU_CTRL_CLK_SEL(CGU_SRC_PLL1);
    LPC_CGU->BASE_UART2_CLK = CGU_CTRL_AUTOBLOCK | CGU_CTRL_CLK_SEL(CGU_SRC_PLL1);
    LPC_CGU->BASE_UART3_CLK = CGU_CTRL_AUTOBLOCK | CGU_CTRL_CLK_SEL(CGU_SRC_PLL1);
    
    LPC_CGU->BASE_SSP0_CLK = CGU_CTRL_AUTOBLOCK | CGU_CTRL_CLK_SEL(CGU_SRC_PLL1);
    LPC_CGU->BASE_SSP1_CLK = CGU_CTRL_AUTOBLOCK | CGU_CTRL_CLK_SEL(CGU_SRC_PLL1);
    
    // Update system core clock variable
    SystemCoreClock = 180000000;  // 180MHz
}

void SystemCoreClockUpdate(void) {
    // In a real implementation, this would read the CGU registers
    // and calculate the actual core clock frequency
    SystemCoreClock = 180000000;  // Fixed at 180MHz for now
}

// System reset function
void SystemReset(void) {
    // Request system reset via NVIC
    #define NVIC_AIRCR_VECTKEY_Pos 16
    #define NVIC_AIRCR_SYSRESETREQ_Pos 2
    #define SCB_AIRCR_VECTKEY_Msk (0xFFFFUL << NVIC_AIRCR_VECTKEY_Pos)
    #define SCB_AIRCR_SYSRESETREQ_Msk (1UL << NVIC_AIRCR_SYSRESETREQ_Pos)
    
    volatile uint32_t *SCB_AIRCR = (uint32_t *)0xE000ED0C;
    
    *SCB_AIRCR = ((0x5FAUL << NVIC_AIRCR_VECTKEY_Pos) |
                 SCB_AIRCR_SYSRESETREQ_Msk);
    
    // Wait for reset
    while (1);
}

// Get M0 core status
uint32_t GetM0CoreStatus(void) {
    // Read M0 status from CREG
    return (LPC_CREG->M0APPMEMMAP & 0x1);
}

// Start M0 core
void StartM0Core(uint32_t m0_image_addr) {
    // Set M0 memory mapping
    LPC_CREG->M0APPMEMMAP = m0_image_addr | 0x1;
    
    // Release M0 reset via RGU (Reset Generation Unit)
    // This is simplified - real implementation would use RGU registers
}

// Stop M0 core  
void StopM0Core(void) {
    // Hold M0 in reset and disable memory mapping
    LPC_CREG->M0APPMEMMAP = 0x0;
}

#endif // HOST_BUILD