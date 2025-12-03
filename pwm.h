#ifndef PWM_H
#define PWM_H

#include <stdint.h>

// Configuration
#define I2C_DEVICE "/dev/i2c-1"
#define PCA9685_ADDR 0x40
#define PWM_FREQ 50.0

// PWM Controller Functions
int init_pwm_controller(void);
void close_pwm_controller(int fd);
int set_pwm(int fd, int channel, uint16_t on, uint16_t off);
int set_pwm_freq(int fd, float freq_hz);
int write_register(int fd, uint8_t reg, uint8_t value);

// Command execution
int execute_pwm_command(int i2c_fd, char *cmd_str);

#endif // PWM_H

