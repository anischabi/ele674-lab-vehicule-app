#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <signal.h>
#include "pwm.h"
#include "imu.h"

// Server Configuration
#define SERVER_IP "0.0.0.0"  // Listen on all interfaces
#define SERVER_PORT 5000

static volatile int running = 1;

// Signal handler for graceful shutdown
void signal_handler(int sig) {
    (void)sig;
    printf("\nShutting down...\n");
    running = 0;
}

// ---------------------------
// Handle client connection
// ---------------------------
void handle_client(int client_fd, int i2c_fd) {
    char buffer[256];
    char response[1024];
    memset(buffer, 0, sizeof(buffer));
    int total = 0;
    int n;

    // Read until newline
    while(total < sizeof(buffer)-1 && (n = read(client_fd, buffer+total, 1)) > 0) {
        if(buffer[total] == '\n') break;
        total += n;
    }
    buffer[total] = '\0';

    if (total > 0) {
        printf("Received: %s\n", buffer);

        // Command routing based on prefix
        if (strncmp(buffer, "IMU", 3) == 0) {
            // IMU command: "IMU <command>"
            // Examples: "IMU read", "IMU raw", "IMU orientation"
            char *imu_cmd = buffer + 3;
            while (*imu_cmd == ' ') imu_cmd++; // Skip spaces
            
            if (execute_imu_command(imu_cmd, response, sizeof(response)) == 0) {
                write(client_fd, response, strlen(response));
            } else {
                write(client_fd, response, strlen(response));
            }
        }
        else if (strncmp(buffer, "PWM", 3) == 0) {
            // PWM command: "PWM <command>"
            // Examples: "PWM 50", "PWM -c 2 60"
            char *pwm_cmd = buffer + 3;
            while (*pwm_cmd == ' ') pwm_cmd++; // Skip spaces
            
            if (execute_pwm_command(i2c_fd, pwm_cmd) == 0) {
                write(client_fd, "OK\n", 3);
            } else {
                write(client_fd, "ERROR\n", 6);
            }
        }
        else {
            // Default: assume PWM command for backward compatibility
            if (execute_pwm_command(i2c_fd, buffer) == 0) {
                write(client_fd, "OK\n", 3);
            } else {
                write(client_fd, "ERROR\n", 6);
            }
        }
    }
}

// ---------------------------
// Main server loop
// ---------------------------
int main() {
    int server_fd, client_fd;
    struct sockaddr_in addr;

    // Setup signal handler
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // Initialize PWM controller
    printf("Initializing PWM controller...\n");
    int i2c_fd = init_pwm_controller();
    if (i2c_fd < 0) {
        fprintf(stderr, "Failed to initialize PWM controller\n");
        return 1;
    }
    printf("PWM controller initialized\n");

    // Initialize IMU controller
    printf("Initializing IMU controller...\n");
    if (init_imu_controller() < 0) {
        fprintf(stderr, "Warning: Failed to initialize IMU controller\n");
        fprintf(stderr, "Server will continue without IMU support\n");
    } else {
        printf("IMU controller initialized\n");
        
        // Start IMU reading thread
        if (start_imu_thread() < 0) {
            fprintf(stderr, "Warning: Failed to start IMU thread\n");
        } else {
            printf("IMU thread started\n");
        }
    }

    // Create server socket
    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        perror("Failed to create socket");
        close_imu_controller();
        close_pwm_controller(i2c_fd);
        return 1;
    }

    // Set socket option to reuse address
    int opt = 1;
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
        perror("setsockopt failed");
    }

    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr(SERVER_IP);
    addr.sin_port = htons(SERVER_PORT);

    if (bind(server_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("Failed to bind socket");
        close(server_fd);
        close_imu_controller();
        close_pwm_controller(i2c_fd);
        return 1;
    }

    if (listen(server_fd, 5) < 0) {
        perror("Failed to listen");
        close(server_fd);
        close_imu_controller();
        close_pwm_controller(i2c_fd);
        return 1;
    }

    printf("Server listening on %s:%d...\n", SERVER_IP, SERVER_PORT);
    printf("Ready to accept commands\n");
    printf("\nCommand formats:\n");
    printf("  PWM commands: <pwm%%> or PWM <pwm%%> or PWM -c <ch> <pwm%%>\n");
    printf("  IMU commands: IMU read | IMU raw | IMU orientation\n");

    // Set accept timeout so we can check 'running' flag
    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    setsockopt(server_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    while(running) {
        client_fd = accept(server_fd, NULL, NULL);
        if (client_fd < 0) {
            if (running) continue;  // Timeout, check again
            else break;  // Shutting down
        }

        handle_client(client_fd, i2c_fd);
        close(client_fd);
    }

    printf("Cleaning up...\n");
    close(server_fd);
    close_imu_controller();
    close_pwm_controller(i2c_fd);
    printf("Server stopped\n");
    
    return 0;
}
