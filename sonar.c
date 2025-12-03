#include "sonar.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <pthread.h>

// Raspberry Pi memory addresses
#define BCM2835_PERI_BASE      0x3F000000  // Pi Zero 2W uses Pi 2/3 base
#define GPIO_BASE_OFFSET       0x200000
#define BLOCK_SIZE             (4*1024)

// GPIO registers
#define GPFSEL0    0
#define GPFSEL1    1
#define GPFSEL2    2
#define GPSET0     7
#define GPCLR0     10
#define GPLEV0     13

// Static variables
static volatile unsigned int *gpio_map = NULL;
static pthread_mutex_t sonar_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_t sonar_thread;
static volatile int thread_running = 0;
static sonar_data_t current_data = {0};

// ---------------------------
// GPIO Direct Access Functions
// ---------------------------
static void gpio_set_input(int pin) {
    int reg = pin / 10;
    int shift = (pin % 10) * 3;
    *(gpio_map + reg) &= ~(7 << shift);
}

static void gpio_set_output(int pin) {
    int reg = pin / 10;
    int shift = (pin % 10) * 3;
    *(gpio_map + reg) &= ~(7 << shift);
    *(gpio_map + reg) |= (1 << shift);
}

static void gpio_write_high(int pin) {
    *(gpio_map + GPSET0) = (1 << pin);
}

static void gpio_write_low(int pin) {
    *(gpio_map + GPCLR0) = (1 << pin);
}

static int gpio_read(int pin) {
    return (*(gpio_map + GPLEV0) & (1 << pin)) ? 1 : 0;
}

static long long get_time_microseconds() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (long long)tv.tv_sec * 1000000LL + tv.tv_usec;
}

// ---------------------------
// Measure distance with HC-SR05
// ---------------------------
static float measure_distance() {
    long long start_time, end_time;
    long pulse_duration;
    float distance;
    int timeout;
    
    // Send 10us pulse to TRIG
    gpio_write_low(SONAR_TRIG_PIN);
    usleep(2);
    gpio_write_high(SONAR_TRIG_PIN);
    usleep(10);
    gpio_write_low(SONAR_TRIG_PIN);
    
    // Wait for ECHO to go HIGH
    timeout = 0;
    while(gpio_read(SONAR_ECHO_PIN) == 0) {
        timeout++;
        if(timeout > 30000) {
            return -1.0f;  // Timeout
        }
        usleep(1);
    }
    start_time = get_time_microseconds();
    
    // Wait for ECHO to go LOW
    timeout = 0;
    while(gpio_read(SONAR_ECHO_PIN) == 1) {
        timeout++;
        if(timeout > 30000) {
            return -1.0f;  // Timeout
        }
        usleep(1);
    }
    end_time = get_time_microseconds();
    
    // Calculate distance
    pulse_duration = end_time - start_time;
    distance = (float)pulse_duration * 0.01715f;  // Speed of sound / 2
    
    return distance;
}

// ---------------------------
// Control LEDs based on distance
// ---------------------------
static void control_leds(const sonar_data_t *data) {
    // Turn off all LEDs first
    gpio_write_low(LED_GREEN);
    gpio_write_low(LED_YELLOW);
    gpio_write_low(LED_RED);
    
    if (!data->valid) {
        // On error, turn off all LEDs
        return;
    }
    
    // Turn on appropriate LED based on distance
    if (data->distance_cm > SONAR_THRESHOLD_HIGH) {
        gpio_write_high(LED_GREEN);  // Far - safe
    } else if (data->distance_cm >= SONAR_THRESHOLD_LOW) {
        gpio_write_high(LED_YELLOW);  // Medium - caution
    } else {
        gpio_write_high(LED_RED);  // Close - danger
    }
}

// ---------------------------
// Determine status based on distance
// ---------------------------
static void update_status(sonar_data_t *data) {
    if (!data->valid) {
        strcpy(data->status, "ERROR");
    } else if (data->distance_cm > SONAR_THRESHOLD_HIGH) {
        strcpy(data->status, "FAR");
    } else if (data->distance_cm >= SONAR_THRESHOLD_LOW) {
        strcpy(data->status, "MEDIUM");
    } else {
        strcpy(data->status, "CLOSE");
    }
}

// ---------------------------
// Thread to continuously read sonar
// ---------------------------
static void* sonar_read_thread(void* arg) {
    (void)arg;
    
    printf("[SONAR] Read thread started\n");
    
    while (thread_running) {
        float distance = measure_distance();
        
        pthread_mutex_lock(&sonar_mutex);
        
        if (distance > 0 && distance < SONAR_MAX_DISTANCE) {
            current_data.distance_cm = distance;
            current_data.valid = 1;
        } else {
            current_data.valid = 0;
        }
        
        update_status(&current_data);
        control_leds(&current_data);  // Update LEDs based on distance
        
        pthread_mutex_unlock(&sonar_mutex);
        
        usleep(1000000 / SONAR_UPDATE_RATE_HZ);
    }
    
    printf("[SONAR] Read thread stopped\n");
    return NULL;
}

// ---------------------------
// Initialize sonar sensor
// ---------------------------
int init_sonar_controller(void) {
    int mem_fd;
    void *gpio_mem;
    
    printf("[SONAR] Initializing HC-SR05...\n");
    
    // Open /dev/mem
    mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (mem_fd < 0) {
        fprintf(stderr, "[SONAR] Error: Cannot open /dev/mem (need root)\n");
        return -1;
    }
    
    // Map GPIO registers
    gpio_mem = mmap(
        NULL,
        BLOCK_SIZE,
        PROT_READ | PROT_WRITE,
        MAP_SHARED,
        mem_fd,
        BCM2835_PERI_BASE + GPIO_BASE_OFFSET
    );
    
    close(mem_fd);
    
    if (gpio_mem == MAP_FAILED) {
        fprintf(stderr, "[SONAR] Error: mmap failed\n");
        return -1;
    }
    
    gpio_map = (volatile unsigned int *)gpio_mem;
    
    // Configure GPIO pins
    gpio_set_output(SONAR_TRIG_PIN);
    gpio_set_input(SONAR_ECHO_PIN);
    gpio_write_low(SONAR_TRIG_PIN);
    
    // Configure LED pins
    gpio_set_output(LED_GREEN);
    gpio_set_output(LED_YELLOW);
    gpio_set_output(LED_RED);
    
    // Turn off all LEDs initially
    gpio_write_low(LED_GREEN);
    gpio_write_low(LED_YELLOW);
    gpio_write_low(LED_RED);
    
    // Stabilization
    printf("[SONAR] Stabilizing sensor...\n");
    sleep(1);
    
    printf("[SONAR] HC-SR05 initialized (TRIG=GPIO%d, ECHO=GPIO%d)\n", 
           SONAR_TRIG_PIN, SONAR_ECHO_PIN);
    printf("[SONAR] LEDs: GREEN=GPIO%d, YELLOW=GPIO%d, RED=GPIO%d\n",
           LED_GREEN, LED_YELLOW, LED_RED);
    return 0;
}

// ---------------------------
// Close sonar sensor
// ---------------------------
void close_sonar_controller(void) {
    stop_sonar_thread();
    
    if (gpio_map) {
        gpio_write_low(SONAR_TRIG_PIN);
        // Turn off all LEDs
        gpio_write_low(LED_GREEN);
        gpio_write_low(LED_YELLOW);
        gpio_write_low(LED_RED);
        
        munmap((void*)gpio_map, BLOCK_SIZE);
        gpio_map = NULL;
    }
    
    printf("[SONAR] Controller closed\n");
}

// ---------------------------
// Start continuous reading thread
// ---------------------------
int start_sonar_thread(void) {
    if (thread_running) {
        fprintf(stderr, "[SONAR] Thread already running\n");
        return -1;
    }
    
    thread_running = 1;
    if (pthread_create(&sonar_thread, NULL, sonar_read_thread, NULL) != 0) {
        perror("[SONAR] Failed to create thread");
        thread_running = 0;
        return -1;
    }
    
    return 0;
}

// ---------------------------
// Stop reading thread
// ---------------------------
void stop_sonar_thread(void) {
    if (thread_running) {
        thread_running = 0;
        pthread_join(sonar_thread, NULL);
    }
}

// ---------------------------
// Get current sonar data (thread-safe)
// ---------------------------
void get_sonar_data(sonar_data_t *data) {
    pthread_mutex_lock(&sonar_mutex);
    memcpy(data, &current_data, sizeof(sonar_data_t));
    pthread_mutex_unlock(&sonar_mutex);
}

// ---------------------------
// Get distance only
// ---------------------------
float get_distance(void) {
    float distance;
    pthread_mutex_lock(&sonar_mutex);
    distance = current_data.valid ? current_data.distance_cm : -1.0f;
    pthread_mutex_unlock(&sonar_mutex);
    return distance;
}

// ---------------------------
// Execute sonar command and format response
// Commands:
//   "read" or "get" - Get distance with status
//   "distance" - Get distance only
//   "status" - Get status only
// ---------------------------
int execute_sonar_command(char *cmd_str, char *response, size_t response_size) {
    if (cmd_str == NULL || response == NULL) {
        return -1;
    }
    
    // Remove trailing whitespace
    char *end = cmd_str + strlen(cmd_str) - 1;
    while (end > cmd_str && (*end == ' ' || *end == '\t' || *end == '\n')) {
        *end = '\0';
        end--;
    }
    
    sonar_data_t data;
    get_sonar_data(&data);
    
    if (strcmp(cmd_str, "read") == 0 || strcmp(cmd_str, "get") == 0 || strcmp(cmd_str, "") == 0) {
        // Full response with status
        if (data.valid) {
            snprintf(response, response_size,
                "{\"distance\":%.2f,\"status\":\"%s\",\"valid\":true}\n",
                data.distance_cm, data.status);
        } else {
            snprintf(response, response_size,
                "{\"distance\":null,\"status\":\"ERROR\",\"valid\":false}\n");
        }
    }
    else if (strcmp(cmd_str, "distance") == 0) {
        // Distance only
        if (data.valid) {
            snprintf(response, response_size, "%.2f cm\n", data.distance_cm);
        } else {
            snprintf(response, response_size, "ERROR\n");
        }
    }
    else if (strcmp(cmd_str, "status") == 0) {
        // Status only
        snprintf(response, response_size, "%s\n", data.status);
    }
    else {
        snprintf(response, response_size, "ERROR: Unknown SONAR command '%s'\n", cmd_str);
        return -1;
    }
    
    return 0;
}
