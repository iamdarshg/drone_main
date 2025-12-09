/*
 * dualcore.c - Drop-in dual-core communication for firmware/Core/dualcore.c
 * LPC4330 M4-M0 inter-core communication and coordination
 */

#ifdef HOST_BUILD
#include <stdio.h>
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>

// Host simulation of dual-core
static pthread_t m0_thread;
static sem_t m4_to_m0_sem;
static sem_t m0_to_m4_sem;
static volatile int m0_running = 0;

// Simulated M0 thread
void* m0_simulation_thread(void* arg) {
    (void)arg;
    printf("M0 Core: Starting simulation thread\n");
    
    while (m0_running) {
        // Wait for M4 signal
        sem_wait(&m4_to_m0_sem);
        
        printf("M0 Core: Processing task\n");
        usleep(10000); // Simulate 10ms work
        
        // Signal M4 completion
        sem_post(&m0_to_m4_sem);
    }
    
    printf("M0 Core: Simulation thread stopping\n");
    return NULL;
}

void dualcore_init(void) {
    printf("Dual-core: Initializing (HOST BUILD)\n");
    
    // Initialize semaphores
    sem_init(&m4_to_m0_sem, 0, 0);
    sem_init(&m0_to_m4_sem, 0, 0);
    
    // Start M0 simulation thread
    m0_running = 1;
    pthread_create(&m0_thread, NULL, m0_simulation_thread, NULL);
    
    printf("Dual-core: M0 simulation started\n");
}

void dualcore_send_to_m0(uint32_t data) {
    printf("Dual-core: M4 -> M0: 0x%08X\n", data);
    sem_post(&m4_to_m0_sem);
}

uint32_t dualcore_receive_from_m0(void) {
    sem_wait(&m0_to_m4_sem);
    printf("Dual-core: M4 <- M0: data received\n");
    return 0x12345678; // Dummy data
}

void dualcore_shutdown(void) {
    printf("Dual-core: Shutting down\n");
    m0_running = 0;
    sem_post(&m4_to_m0_sem); // Wake up M0 thread to exit
    pthread_join(m0_thread, NULL);
    
    sem_destroy(&m4_to_m0_sem);
    sem_destroy(&m0_to_m4_sem);
}

#else
// Real LPC4330 dual-core implementation

#include <stdint.h>

// LPC4330 inter-core communication registers
#define LPC_CREG_BASE       0x40043000
#define LPC_M0_IMAGE_BASE   0x10080000  // M0 code location in RAM

// Mailbox registers for M4-M0 communication
typedef struct {
    volatile uint32_t IRQ;          // Interrupt request
    volatile uint32_t IRQSET;       // Set interrupt
    volatile uint32_t IRQCLR;       // Clear interrupt
    volatile uint32_t reserved;
} MBOX_T;

#define LPC_MBOX_M4         ((MBOX_T *) 0x400C0000)
#define LPC_MBOX_M0         ((MBOX_T *) 0x400C0004)

// Shared memory for inter-core communication
#define SHARED_MEM_BASE     0x10088000
#define SHARED_MEM_SIZE     0x1000      // 4KB shared memory

typedef struct {
    volatile uint32_t m4_to_m0_cmd;
    volatile uint32_t m4_to_m0_data[16];
    volatile uint32_t m0_to_m4_status;
    volatile uint32_t m0_to_m4_data[16];
    volatile uint32_t mailbox_lock;
    volatile uint32_t sequence_number;
} shared_mem_t;

static volatile shared_mem_t *shared_mem = (shared_mem_t *)SHARED_MEM_BASE;

// M0 commands
#define M0_CMD_NONE         0
#define M0_CMD_WATCHDOG     1
#define M0_CMD_MONITOR      2
#define M0_CMD_SELFTEST     3
#define M0_CMD_SHUTDOWN     4

// M0 status
#define M0_STATUS_IDLE      0
#define M0_STATUS_BUSY      1
#define M0_STATUS_READY     2
#define M0_STATUS_ERROR     3

// External M0 image (linked separately)
extern const uint32_t m0_image_start[];
extern const uint32_t m0_image_size;

void dualcore_init(void) {
    // Initialize shared memory
    volatile uint32_t *shared_ptr = (volatile uint32_t *)SHARED_MEM_BASE;
    for (int i = 0; i < (SHARED_MEM_SIZE / 4); i++) {
        shared_ptr[i] = 0;
    }
    
    shared_mem->m4_to_m0_cmd = M0_CMD_NONE;
    shared_mem->m0_to_m4_status = M0_STATUS_IDLE;
    shared_mem->mailbox_lock = 0;
    shared_mem->sequence_number = 0;
    
    // Copy M0 image to RAM
    if (m0_image_size > 0) {
        const uint32_t *src = m0_image_start;
        volatile uint32_t *dst = (volatile uint32_t *)LPC_M0_IMAGE_BASE;
        
        for (uint32_t i = 0; i < (m0_image_size / 4); i++) {
            dst[i] = src[i];
        }
    }
    
    // Configure M0 memory mapping
    extern void StartM0Core(uint32_t m0_image_addr);
    StartM0Core(LPC_M0_IMAGE_BASE);
    
    // Wait for M0 to be ready
    int timeout = 10000;
    while (shared_mem->m0_to_m4_status != M0_STATUS_READY && timeout > 0) {
        timeout--;
        for (volatile int i = 0; i < 1000; i++); // Delay
    }
    
    if (timeout == 0) {
        // M0 startup failed
        return;
    }
}

void dualcore_send_to_m0(uint32_t command) {
    // Wait for M0 to be ready
    while (shared_mem->m0_to_m4_status == M0_STATUS_BUSY);
    
    // Acquire mailbox lock
    while (__sync_lock_test_and_set(&shared_mem->mailbox_lock, 1));
    
    // Send command
    shared_mem->m4_to_m0_cmd = command;
    shared_mem->sequence_number++;
    
    // Release lock
    shared_mem->mailbox_lock = 0;
    
    // Trigger M0 interrupt
    LPC_MBOX_M0->IRQSET = 1;
}

uint32_t dualcore_receive_from_m0(void) {
    // Wait for M0 response
    while (shared_mem->m0_to_m4_status != M0_STATUS_READY);
    
    return shared_mem->m0_to_m4_data[0];
}

// Send data array to M0
void dualcore_send_data_to_m0(const uint32_t *data, int count) {
    if (count > 16) count = 16; // Limit to shared memory size
    
    // Wait for M0 to be ready
    while (shared_mem->m0_to_m4_status == M0_STATUS_BUSY);
    
    // Acquire mailbox lock
    while (__sync_lock_test_and_set(&shared_mem->mailbox_lock, 1));
    
    // Copy data
    for (int i = 0; i < count; i++) {
        shared_mem->m4_to_m0_data[i] = data[i];
    }
    
    // Release lock
    shared_mem->mailbox_lock = 0;
}

// Receive data array from M0
int dualcore_receive_data_from_m0(uint32_t *data, int max_count) {
    if (max_count > 16) max_count = 16;
    
    // Wait for M0 response
    while (shared_mem->m0_to_m4_status != M0_STATUS_READY);
    
    // Copy data
    for (int i = 0; i < max_count; i++) {
        data[i] = shared_mem->m0_to_m4_data[i];
    }
    
    return max_count;
}

// Check if M0 is running
bool dualcore_m0_is_running(void) {
    return (shared_mem->m0_to_m4_status != M0_STATUS_IDLE);
}

// Get M0 status
uint32_t dualcore_get_m0_status(void) {
    return shared_mem->m0_to_m4_status;
}

// M4 interrupt handler for M0 messages
void M0CORE_IRQHandler(void) {
    // Clear interrupt
    LPC_MBOX_M4->IRQCLR = 1;
    
    // Process M0 message
    if (shared_mem->m0_to_m4_status == M0_STATUS_READY) {
        // M0 has completed a task
        // Handle response if needed
    }
}

// Shutdown M0 core
void dualcore_shutdown(void) {
    // Send shutdown command
    dualcore_send_to_m0(M0_CMD_SHUTDOWN);
    
    // Wait for M0 to acknowledge
    int timeout = 1000;
    while (shared_mem->m0_to_m4_status != M0_STATUS_IDLE && timeout > 0) {
        timeout--;
        for (volatile int i = 0; i < 1000; i++);
    }
    
    // Stop M0 core
    extern void StopM0Core(void);
    StopM0Core();
}

// High-level command functions
void dualcore_trigger_watchdog(void) {
    dualcore_send_to_m0(M0_CMD_WATCHDOG);
}

void dualcore_trigger_monitor(void) {
    dualcore_send_to_m0(M0_CMD_MONITOR);
}

void dualcore_trigger_selftest(void) {
    dualcore_send_to_m0(M0_CMD_SELFTEST);
}

#endif // HOST_BUILD
