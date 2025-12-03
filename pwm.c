#include "pwm.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

// ---------------------------
// Write a single byte to a PCA9685 register
// ---------------------------
int write_register(int fd, uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = {reg, value};
    if (write(fd, buffer, 2) != 2) {
        perror("Failed to write register");
        return -1;
    }
    return 0;
}

// ---------------------------
// Set PWM values for a channel
// ---------------------------
int set_pwm(int fd, int channel, uint16_t on, uint16_t off) {
    int base = 0x06 + 4 * channel;
    if (write_register(fd, base, on & 0xFF) < 0) return -1;
    if (write_register(fd, base + 1, (on >> 8) & 0x0F) < 0) return -1;
    if (write_register(fd, base + 2, off & 0xFF) < 0) return -1;
    if (write_register(fd, base + 3, (off >> 8) & 0x0F) < 0) return -1;
    return 0;
}

// ---------------------------
// Set PWM frequency (Hz)
// ---------------------------
int set_pwm_freq(int fd, float freq_hz) {
    if (freq_hz < 24) freq_hz = 24;
    if (freq_hz > 1526) freq_hz = 1526;

    float prescaleval = 25000000.0 / (4096.0 * freq_hz) - 1.0;
    uint8_t prescale = (uint8_t)(prescaleval + 0.5);

    uint8_t oldmode = 0;
    read(fd, &oldmode, 1);

    write_register(fd, 0x00, 0x10);
    write_register(fd, 0xFE, prescale);
    write_register(fd, 0x00, 0x00);
    usleep(5000);
    return 0;
}

// ---------------------------
// Initialize I2C and PCA9685
// ---------------------------
int init_pwm_controller(void) {
    int fd = open(I2C_DEVICE, O_RDWR);
    if (fd < 0) {
        perror("Failed to open I2C device");
        return -1;
    }

    if (ioctl(fd, I2C_SLAVE, PCA9685_ADDR) < 0) {
        perror("Failed to set I2C address");
        close(fd);
        return -1;
    }

    if (set_pwm_freq(fd, PWM_FREQ) < 0) {
        fprintf(stderr, "Failed to set PWM frequency\n");
        close(fd);
        return -1;
    }

    return fd;
}

// ---------------------------
// Close PWM controller
// ---------------------------
void close_pwm_controller(int fd) {
    if (fd >= 0) {
        close(fd);
    }
}

// ---------------------------
// Parse and execute PWM command
// Format: "<pwm1%>" or "<pwm1%> <pwm2%>" or "-c <ch> <pwm%>" or "-t <time> <pwm%>"
// ---------------------------
int execute_pwm_command(int i2c_fd, char *cmd_str) {
    char *tokens[10];
    int token_count = 0;
    
    // Tokenize the command
    char *token = strtok(cmd_str, " \t\n");
    while (token != NULL && token_count < 10) {
        tokens[token_count++] = token;
        token = strtok(NULL, " \t\n");
    }

    if (token_count == 0) {
        fprintf(stderr, "Empty command\n");
        return -1;
    }

    int duration = 0;
    int channel = 0;
    int use_dual = 0;
    float pwm1 = 0, pwm2 = 0;
    int custom_channel = 0;
    const int max_count = 4095;

    // Parse arguments
    int i = 0;
    while (i < token_count) {
        if (strcmp(tokens[i], "-t") == 0) {
            if (i + 1 >= token_count) {
                fprintf(stderr, "Missing timeout value\n");
                return -1;
            }
            duration = atoi(tokens[i + 1]);
            i += 2;
        } else if (strcmp(tokens[i], "-c") == 0) {
            if (i + 1 >= token_count) {
                fprintf(stderr, "Missing channel value\n");
                return -1;
            }
            channel = atoi(tokens[i + 1]);
            custom_channel = 1;
            i += 2;
        } else {
            // PWM values
            pwm1 = atof(tokens[i]);
            if (i + 1 < token_count && tokens[i + 1][0] != '-') {
                pwm2 = atof(tokens[i + 1]);
                use_dual = 1;
                i += 2;
            } else {
                i++;
            }
            break;
        }
    }

    // Validate PWM
    if (pwm1 < 0 || pwm1 > 100 || (use_dual && (pwm2 < 0 || pwm2 > 100))) {
        fprintf(stderr, "Error: PWM must be between 0 and 100\n");
        return -1;
    }

    // Execute PWM command
    if (use_dual && !custom_channel) {
        // Set channels 0 and 1
        uint16_t off1 = (uint16_t)((pwm1 / 100.0) * max_count);
        uint16_t off2 = (uint16_t)((pwm2 / 100.0) * max_count);

        printf("Setting Ch0=%.1f%%, Ch1=%.1f%%", pwm1, pwm2);
        if (duration > 0) printf(" for %ds", duration);
        printf("\n");

        set_pwm(i2c_fd, 0, 0, off1);
        set_pwm(i2c_fd, 1, 0, off2);

        if (duration > 0) {
            sleep(duration);
            printf("Stopping PWM\n");
            set_pwm(i2c_fd, 0, 0, 0);
            set_pwm(i2c_fd, 1, 0, 0);
        }
    } else {
        // Single channel
        uint16_t off1 = (uint16_t)((pwm1 / 100.0) * max_count);

        printf("Setting Ch%d=%.1f%%", channel, pwm1);
        if (duration > 0) printf(" for %ds", duration);
        printf("\n");

        set_pwm(i2c_fd, channel, 0, off1);

        if (duration > 0) {
            sleep(duration);
            printf("Stopping PWM\n");
            set_pwm(i2c_fd, channel, 0, 0);
        }
    }

    return 0;
}
