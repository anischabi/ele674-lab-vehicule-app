#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include <pthread.h>

// Configuration
#define IMU_I2C_DEVICE "/dev/i2c-1"
#define IMU_UPDATE_RATE_HZ 10

// Structure to store sensor data
typedef struct {
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    float mag_x, mag_y, mag_z;
    float temp;
    float roll, pitch, yaw;
} imu_data_t;

// IMU Controller Functions
int init_imu_controller(void);
void close_imu_controller(void);
int start_imu_thread(void);
void stop_imu_thread(void);

// Data access (thread-safe)
void get_imu_data(imu_data_t *data);

// Command execution
int execute_imu_command(char *cmd_str, char *response, size_t response_size);

#endif // IMU_H
