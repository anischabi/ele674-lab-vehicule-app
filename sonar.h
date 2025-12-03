#ifndef SONAR_H
#define SONAR_H

#include <stdint.h>
#include <pthread.h>

// Configuration
#define SONAR_TRIG_PIN 27
#define SONAR_ECHO_PIN 17
#define SONAR_UPDATE_RATE_HZ 10
#define SONAR_MAX_DISTANCE 400.0f  // cm
#define SONAR_MIN_DISTANCE 2.0f    // cm

// LED pins
#define LED_GREEN  25  // GPIO25 - Far (>60cm)
#define LED_YELLOW 24  // GPIO24 - Medium (20-60cm)
#define LED_RED    23  // GPIO23 - Close (<20cm)

// Distance thresholds
#define SONAR_THRESHOLD_HIGH 60.0f  // cm
#define SONAR_THRESHOLD_LOW  20.0f  // cm

// Structure to store sonar data
typedef struct {
    float distance_cm;
    int valid;  // 1 if measurement is valid, 0 otherwise
    char status[32];  // "FAR", "MEDIUM", "CLOSE", "ERROR"
} sonar_data_t;

// Sonar Controller Functions
int init_sonar_controller(void);
void close_sonar_controller(void);
int start_sonar_thread(void);
void stop_sonar_thread(void);

// Data access (thread-safe)
void get_sonar_data(sonar_data_t *data);
float get_distance(void);

// Command execution
int execute_sonar_command(char *cmd_str, char *response, size_t response_size);

#endif // SONAR_H

