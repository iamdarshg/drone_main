/*
 * syscalls.c - Drop-in system calls for firmware/Core/syscalls.c  
 * Newlib syscall implementations for embedded systems
 */

#ifdef HOST_BUILD
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/times.h>
#include <errno.h>

// Host build - use standard library calls
int _write(int file, char *ptr, int len) {
    return write(file, ptr, len);
}

int _read(int file, char *ptr, int len) {
    return read(file, ptr, len);
}

void *_sbrk(int incr) {
    return sbrk(incr);
}

int _close(int file) {
    return close(file);
}

int _fstat(int file, struct stat *st) {
    return fstat(file, st);
}

int _isatty(int file) {
    return isatty(file);
}

int _lseek(int file, int ptr, int dir) {
    return lseek(file, ptr, dir);
}

#else
// Embedded syscalls for LPC4330

#include <errno.h>
#include <sys/stat.h>
#include <sys/times.h>
#include <sys/unistd.h>
#include <stdint.h>

#undef errno
extern int errno;

// Forward declarations
extern void uart0_putchar(char c);
extern int uart0_getchar(void);
extern void *heap_malloc(size_t size);
extern void heap_free(void *ptr);

// Heap management
extern char _end;        // End of BSS, start of heap
extern char _estack;     // End of stack
static char *heap_ptr = NULL;

// Initialize heap pointer
void _init_heap(void) {
    if (heap_ptr == NULL) {
        heap_ptr = &_end;
    }
}

// Write system call - redirect to UART
int _write(int file, char *ptr, int len) {
    int i;
    
    // File descriptors 1 (stdout) and 2 (stderr) go to UART
    if (file == STDOUT_FILENO || file == STDERR_FILENO) {
        for (i = 0; i < len; i++) {
            uart0_putchar(ptr[i]);
        }
        return len;
    }
    
    // Other files not supported
    errno = EBADF;
    return -1;
}

// Read system call - redirect from UART  
int _read(int file, char *ptr, int len) {
    int i;
    
    // File descriptor 0 (stdin) comes from UART
    if (file == STDIN_FILENO) {
        for (i = 0; i < len; i++) {
            ptr[i] = uart0_getchar();
            if (ptr[i] == '\r' || ptr[i] == '\n') {
                ptr[i] = '\n';
                i++;
                break;
            }
        }
        return i;
    }
    
    // Other files not supported
    errno = EBADF;
    return -1;
}

// Heap allocation (sbrk)
void *_sbrk(int incr) {
    char *prev_heap_ptr;
    
    _init_heap();
    
    prev_heap_ptr = heap_ptr;
    
    // Check if we have enough space
    if (heap_ptr + incr > &_estack) {
        errno = ENOMEM;
        return (void *)-1;
    }
    
    heap_ptr += incr;
    return (void *)prev_heap_ptr;
}

// Close file
int _close(int file) {
    // We don't support file operations
    (void)file;
    return -1;
}

// File status
int _fstat(int file, struct stat *st) {
    // Assume character devices for stdio
    if (file >= 0 && file <= 2) {
        st->st_mode = S_IFCHR;
        return 0;
    }
    
    errno = EBADF;
    return -1;
}

// Check if file is a terminal
int _isatty(int file) {
    // Standard streams are terminal-like (UART)
    if (file >= 0 && file <= 2) {
        return 1;
    }
    
    errno = EBADF;
    return 0;
}

// Seek in file
int _lseek(int file, int ptr, int dir) {
    // We don't support seeking
    (void)file;
    (void)ptr;
    (void)dir;
    
    errno = ESPIPE;
    return -1;
}

// Open file
int _open(const char *name, int flags, int mode) {
    // We don't support file operations
    (void)name;
    (void)flags;
    (void)mode;
    
    errno = ENOENT;
    return -1;
}

// Wait for child process
int _wait(int *status) {
    // We don't support processes
    (void)status;
    
    errno = ECHILD;
    return -1;
}

// Unlink file
int _unlink(const char *name) {
    // We don't support file operations
    (void)name;
    
    errno = ENOENT;
    return -1;
}

// Get process times
int _times(struct tms *buf) {
    // Return system tick count
    extern uint32_t get_system_tick(void);
    
    if (buf) {
        buf->tms_utime = get_system_tick();
        buf->tms_stime = 0;
        buf->tms_cutime = 0;
        buf->tms_cstime = 0;
    }
    
    return get_system_tick();
}

// Get process status
int _stat(const char *file, struct stat *st) {
    // We don't support file operations
    (void)file;
    (void)st;
    
    errno = ENOENT;
    return -1;
}

// Create link
int _link(const char *old, const char *new) {
    // We don't support file operations
    (void)old;
    (void)new;
    
    errno = EMLINK;
    return -1;
}

// Fork process
int _fork(void) {
    // We don't support processes
    errno = EAGAIN;
    return -1;
}

// Execute program
int _execve(const char *name, char * const *argv, char * const *env) {
    // We don't support processes
    (void)name;
    (void)argv;
    (void)env;
    
    errno = ENOMEM;
    return -1;
}

// Get process ID
int _getpid(void) {
    return 1; // Always return PID 1
}

// Kill process
int _kill(int pid, int sig) {
    // We don't support signals
    (void)pid;
    (void)sig;
    
    errno = EINVAL;
    return -1;
}

// Exit process
void _exit(int status) {
    // Disable interrupts and loop forever
    __disable_irq();
    
    // Optional: reset system
    extern void SystemReset(void);
    SystemReset();
    
    // Should never reach here
    while (1) {
        __asm__("wfi");
    }
}

// System call stubs that may be needed by some libraries
void _fini(void) {
    // Destructor stub
}

void _init(void) {
    // Constructor stub
}

// Memory allocation wrappers (optional)
void *malloc(size_t size) {
    return _sbrk(size);
}

void free(void *ptr) {
    // Simple heap doesn't support free
    (void)ptr;
}

void *realloc(void *ptr, size_t size) {
    // Simple implementation
    void *new_ptr = malloc(size);
    if (new_ptr && ptr) {
        // Copy old data (we don't know the old size, so this is unsafe)
        // In production, use a proper heap implementation
    }
    return new_ptr;
}

void *calloc(size_t nmemb, size_t size) {
    size_t total = nmemb * size;
    void *ptr = malloc(total);
    if (ptr) {
        char *p = (char *)ptr;
        for (size_t i = 0; i < total; i++) {
            p[i] = 0;
        }
    }
    return ptr;
}

// Get heap statistics
size_t get_heap_free(void) {
    _init_heap();
    return &_estack - heap_ptr;
}

size_t get_heap_used(void) {
    _init_heap();
    return heap_ptr - &_end;
}

#endif // HOST_BUILD