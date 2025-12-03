#include "imu.h"
#include "lsm9ds1.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <pthread.h>

// Static variables
static lsm9ds1_t sensor;
static pthread_mutex_t sensor_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_t sensor_thread;
static volatile int thread_running = 0;
static imu_data_t current_data = {0};

// ---------------------------
// Calculate roll, pitch, yaw from sensor data
// ---------------------------
static void calculate_orientation(imu_data_t *data) {
    // Roll (rotation around X axis)
    data->roll = atan2(data->accel_y, 
                       sqrt(data->accel_x * data->accel_x + 
                            data->accel_z * data->accel_z));
    
    // Pitch (rotation around Y axis)
    data->pitch = atan2(-data->accel_x,
                        sqrt(data->accel_y * data->accel_y + 
                             data->accel_z * data->accel_z));
    
    // Yaw from magnetometer (simplified, without tilt compensation)
    data->yaw = atan2(data->mag_y, data->mag_x);
    
    // Convert to degrees
    data->roll *= 180.0 / M_PI;
    data->pitch *= 180.0 / M_PI;
    data->yaw *= 180.0 / M_PI;
}

// ---------------------------
// Thread to continuously read sensor
// ---------------------------
static void* imu_read_thread(void* arg) {
    (void)arg; // Unused
    
    printf("[IMU] Read thread started\n");
    
    while (thread_running) {
        if (lsm9ds1_read(&sensor)) {
            pthread_mutex_lock(&sensor_mutex);
            
            current_data.accel_x = sensor.acceleration.x;
            current_data.accel_y = sensor.acceleration.y;
            current_data.accel_z = sensor.acceleration.z;
            
            current_data.gyro_x = sensor.gyro.x;
            current_data.gyro_y = sensor.gyro.y;
            current_data.gyro_z = sensor.gyro.z;
            
            current_data.mag_x = sensor.magnetic.x;
            current_data.mag_y = sensor.magnetic.y;
            current_data.mag_z = sensor.magnetic.z;
            
            current_data.temp = sensor.temperature;
            
            calculate_orientation(&current_data);
            
            pthread_mutex_unlock(&sensor_mutex);
        }
        
        usleep(1000000 / IMU_UPDATE_RATE_HZ);
    }
    
    printf("[IMU] Read thread stopped\n");
    return NULL;
}

// ---------------------------
// Initialize IMU sensor
// ---------------------------
int init_imu_controller(void) {
    printf("[IMU] Initializing LSM9DS1...\n");
    
    if (!lsm9ds1_init(&sensor, IMU_I2C_DEVICE)) {
        fprintf(stderr, "[IMU] Failed to initialize LSM9DS1\n");
        return -1;
    }
    
    // Configure sensor
    lsm9ds1_setup_accel(&sensor, LSM9DS1_ACCELRANGE_2G, LSM9DS1_ACCELDATARATE_119HZ);
    lsm9ds1_setup_gyro(&sensor, LSM9DS1_GYROSCALE_245DPS);
    lsm9ds1_setup_mag(&sensor, LSM9DS1_MAGGAIN_4GAUSS);
    
    printf("[IMU] LSM9DS1 initialized successfully\n");
    return 0;
}

// ---------------------------
// Close IMU sensor
// ---------------------------
void close_imu_controller(void) {
    stop_imu_thread();
    lsm9ds1_close(&sensor);
    printf("[IMU] Controller closed\n");
}

// ---------------------------
// Start continuous reading thread
// ---------------------------
int start_imu_thread(void) {
    if (thread_running) {
        fprintf(stderr, "[IMU] Thread already running\n");
        return -1;
    }
    
    thread_running = 1;
    if (pthread_create(&sensor_thread, NULL, imu_read_thread, NULL) != 0) {
        perror("[IMU] Failed to create thread");
        thread_running = 0;
        return -1;
    }
    
    return 0;
}

// ---------------------------
// Stop reading thread
// ---------------------------
void stop_imu_thread(void) {
    if (thread_running) {
        thread_running = 0;
        pthread_join(sensor_thread, NULL);
    }
}

// ---------------------------
// Get current IMU data (thread-safe)
// ---------------------------
void get_imu_data(imu_data_t *data) {
    pthread_mutex_lock(&sensor_mutex);
    memcpy(data, &current_data, sizeof(imu_data_t));
    pthread_mutex_unlock(&sensor_mutex);
}

// ---------------------------
// Execute IMU command and format response
// Commands:
//   "read" or "get" - Get current sensor data in JSON format
//   "raw" - Get raw sensor values
//   "orientation" - Get only roll, pitch, yaw
// ---------------------------
int execute_imu_command(char *cmd_str, char *response, size_t response_size) {
    if (cmd_str == NULL || response == NULL) {
        return -1;
    }
    
    // Remove trailing whitespace
    char *end = cmd_str + strlen(cmd_str) - 1;
    while (end > cmd_str && (*end == ' ' || *end == '\t' || *end == '\n')) {
        *end = '\0';
        end--;
    }
    
    imu_data_t data;
    get_imu_data(&data);
    
    if (strcmp(cmd_str, "read") == 0 || strcmp(cmd_str, "get") == 0 || strcmp(cmd_str, "") == 0) {
        // Full JSON response
        snprintf(response, response_size,
            "{\"accel\":[%.3f,%.3f,%.3f],"
            "\"gyro\":[%.3f,%.3f,%.3f],"
            "\"mag\":[%.3f,%.3f,%.3f],"
            "\"temp\":%.1f,"
            "\"roll\":%.1f,\"pitch\":%.1f,\"yaw\":%.1f}\n",
            data.accel_x, data.accel_y, data.accel_z,
            data.gyro_x, data.gyro_y, data.gyro_z,
            data.mag_x, data.mag_y, data.mag_z,
            data.temp,
            data.roll, data.pitch, data.yaw);
    }
    else if (strcmp(cmd_str, "raw") == 0) {
        // Raw sensor values
        snprintf(response, response_size,
            "Accel: %.3f %.3f %.3f | Gyro: %.3f %.3f %.3f | Mag: %.3f %.3f %.3f | Temp: %.1f°C\n",
            data.accel_x, data.accel_y, data.accel_z,
            data.gyro_x, data.gyro_y, data.gyro_z,
            data.mag_x, data.mag_y, data.mag_z,
            data.temp);
    }
    else if (strcmp(cmd_str, "orientation") == 0) {
        // Only orientation
        snprintf(response, response_size,
            "Roll: %.1f° | Pitch: %.1f° | Yaw: %.1f°\n",
            data.roll, data.pitch, data.yaw);
    }
    else {
        snprintf(response, response_size, "ERROR: Unknown IMU command '%s'\n", cmd_str);
        return -1;
    }
    
    return 0;
}
