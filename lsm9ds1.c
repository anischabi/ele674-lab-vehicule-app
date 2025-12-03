#include "lsm9ds1.h"
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <math.h>

#define SENSORS_GRAVITY_STANDARD 9.80665f
#define SENSORS_DPS_TO_RADS      0.017453293f

// Facteurs de conversion LSB
#define LSM9DS1_ACCEL_MG_LSB_2G   0.061f
#define LSM9DS1_ACCEL_MG_LSB_4G   0.122f
#define LSM9DS1_ACCEL_MG_LSB_8G   0.244f
#define LSM9DS1_ACCEL_MG_LSB_16G  0.732f

#define LSM9DS1_GYRO_DPS_DIGIT_245DPS   0.00875f
#define LSM9DS1_GYRO_DPS_DIGIT_500DPS   0.01750f
#define LSM9DS1_GYRO_DPS_DIGIT_2000DPS  0.07000f

// Fonctions utilitaires I2C
static int i2c_open_device(const char *bus, int addr) {
    int fd = open(bus, O_RDWR);
    if (fd < 0) {
        perror("Erreur ouverture bus I2C");
        return -1;
    }
    
    if (ioctl(fd, I2C_SLAVE, addr) < 0) {
        perror("Erreur configuration adresse I2C");
        close(fd);
        return -1;
    }
    
    return fd;
}

static int i2c_write_byte(int fd, uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = {reg, value};
    if (write(fd, buffer, 2) != 2) {
        perror("Erreur écriture I2C");
        return -1;
    }
    return 0;
}

static int i2c_read_byte(int fd, uint8_t reg, uint8_t *value) {
    if (write(fd, &reg, 1) != 1) {
        perror("Erreur écriture registre I2C");
        return -1;
    }
    if (read(fd, value, 1) != 1) {
        perror("Erreur lecture I2C");
        return -1;
    }
    return 0;
}

static int i2c_read_block(int fd, uint8_t reg, uint8_t *buffer, size_t len) {
    // Pour lire plusieurs registres contigus, on utilise l'auto-increment
    reg |= 0x80;
    
    if (write(fd, &reg, 1) != 1) {
        perror("Erreur écriture registre I2C");
        return -1;
    }
    if (read(fd, buffer, len) != len) {
        perror("Erreur lecture bloc I2C");
        return -1;
    }
    return 0;
}

// Initialisation
bool lsm9ds1_init(lsm9ds1_t *lsm, const char *i2c_bus) {
    // Ouvrir les deux devices I2C
    lsm->fd_xg = i2c_open_device(i2c_bus, LSM9DS1_ADDRESS_ACCELGYRO);
    if (lsm->fd_xg < 0) {
        printf("Erreur: Impossible d'ouvrir accel/gyro\n");
        return false;
    }
    
    lsm->fd_mag = i2c_open_device(i2c_bus, LSM9DS1_ADDRESS_MAG);
    if (lsm->fd_mag < 0) {
        printf("Erreur: Impossible d'ouvrir magnétomètre\n");
        close(lsm->fd_xg);
        return false;
    }
    
    // Vérifier les WHO_AM_I
    uint8_t id;
    if (i2c_read_byte(lsm->fd_xg, LSM9DS1_REGISTER_WHO_AM_I_XG, &id) < 0) {
        printf("Erreur lecture WHO_AM_I accel/gyro\n");
        return false;
    }
    if (id != LSM9DS1_XG_ID) {
        printf("ID accel/gyro incorrect: 0x%02X (attendu: 0x%02X)\n", id, LSM9DS1_XG_ID);
        return false;
    }
    
    if (i2c_read_byte(lsm->fd_mag, LIS3MDL_REGISTER_WHO_AM_I, &id) < 0) {
        printf("Erreur lecture WHO_AM_I mag\n");
        return false;
    }
    if (id != LIS3MDL_ID) {
        printf("ID magnétomètre incorrect: 0x%02X (attendu: 0x%02X)\n", id, LIS3MDL_ID);
        return false;
    }
    
    // Soft reset
    i2c_write_byte(lsm->fd_xg, LSM9DS1_REGISTER_CTRL_REG8, 0x05);
    usleep(10000); // 10ms
    
    // Activer le gyroscope
    i2c_write_byte(lsm->fd_xg, LSM9DS1_REGISTER_CTRL_REG1_G, 0xC0);
    
    // Activer l'accéléromètre
    i2c_write_byte(lsm->fd_xg, LSM9DS1_REGISTER_CTRL_REG5_XL, 0x38);
    i2c_write_byte(lsm->fd_xg, LSM9DS1_REGISTER_CTRL_REG6_XL, 0xC0);
    
    // Activer le magnétomètre en mode continu
    i2c_write_byte(lsm->fd_mag, LIS3MDL_REGISTER_CTRL_REG3, 0x00);
    
    // Configuration par défaut
    lsm9ds1_setup_accel(lsm, LSM9DS1_ACCELRANGE_2G, LSM9DS1_ACCELDATARATE_10HZ);
    lsm9ds1_setup_gyro(lsm, LSM9DS1_GYROSCALE_245DPS);
    lsm9ds1_setup_mag(lsm, LSM9DS1_MAGGAIN_4GAUSS);
    
    return true;
}

void lsm9ds1_close(lsm9ds1_t *lsm) {
    if (lsm->fd_xg >= 0) close(lsm->fd_xg);
    if (lsm->fd_mag >= 0) close(lsm->fd_mag);
}

void lsm9ds1_setup_accel(lsm9ds1_t *lsm, lsm9ds1_accel_range_t range, lsm9ds1_accel_datarate_t rate) {
    uint8_t reg;
    i2c_read_byte(lsm->fd_xg, LSM9DS1_REGISTER_CTRL_REG6_XL, &reg);
    reg &= ~0b11111000;
    reg |= range | rate;
    i2c_write_byte(lsm->fd_xg, LSM9DS1_REGISTER_CTRL_REG6_XL, reg);
    
    // Mettre à jour le facteur de conversion
    switch(range) {
        case LSM9DS1_ACCELRANGE_2G:
            lsm->accel_mg_lsb = LSM9DS1_ACCEL_MG_LSB_2G;
            break;
        case LSM9DS1_ACCELRANGE_4G:
            lsm->accel_mg_lsb = LSM9DS1_ACCEL_MG_LSB_4G;
            break;
        case LSM9DS1_ACCELRANGE_8G:
            lsm->accel_mg_lsb = LSM9DS1_ACCEL_MG_LSB_8G;
            break;
        case LSM9DS1_ACCELRANGE_16G:
            lsm->accel_mg_lsb = LSM9DS1_ACCEL_MG_LSB_16G;
            break;
    }
}

void lsm9ds1_setup_gyro(lsm9ds1_t *lsm, lsm9ds1_gyro_scale_t scale) {
    uint8_t reg;
    i2c_read_byte(lsm->fd_xg, LSM9DS1_REGISTER_CTRL_REG1_G, &reg);
    reg &= ~0b00011000;
    reg |= scale;
    i2c_write_byte(lsm->fd_xg, LSM9DS1_REGISTER_CTRL_REG1_G, reg);
    
    switch(scale) {
        case LSM9DS1_GYROSCALE_245DPS:
            lsm->gyro_dps_digit = LSM9DS1_GYRO_DPS_DIGIT_245DPS;
            break;
        case LSM9DS1_GYROSCALE_500DPS:
            lsm->gyro_dps_digit = LSM9DS1_GYRO_DPS_DIGIT_500DPS;
            break;
        case LSM9DS1_GYROSCALE_2000DPS:
            lsm->gyro_dps_digit = LSM9DS1_GYRO_DPS_DIGIT_2000DPS;
            break;
    }
}

void lsm9ds1_setup_mag(lsm9ds1_t *lsm, lsm9ds1_mag_gain_t gain) {
    uint8_t reg_value = (gain & 0x03) << 5;
    i2c_write_byte(lsm->fd_mag, LIS3MDL_REGISTER_CTRL_REG2, reg_value);
    
    // Facteurs de conversion pour LIS3MDL
    switch(gain) {
        case LSM9DS1_MAGGAIN_4GAUSS:
            lsm->mag_gauss_lsb = 1.0f / 6842.0f;
            break;
        case LSM9DS1_MAGGAIN_8GAUSS:
            lsm->mag_gauss_lsb = 1.0f / 3421.0f;
            break;
        case LSM9DS1_MAGGAIN_12GAUSS:
            lsm->mag_gauss_lsb = 1.0f / 2281.0f;
            break;
        case LSM9DS1_MAGGAIN_16GAUSS:
            lsm->mag_gauss_lsb = 1.0f / 1711.0f;
            break;
    }
}

bool lsm9ds1_read(lsm9ds1_t *lsm) {
    uint8_t buffer[6];
    
    // Lire l'accéléromètre
    if (i2c_read_block(lsm->fd_xg, LSM9DS1_REGISTER_OUT_X_L_XL, buffer, 6) < 0) {
        return false;
    }
    lsm->accel_raw[0] = (int16_t)((buffer[1] << 8) | buffer[0]);
    lsm->accel_raw[1] = (int16_t)((buffer[3] << 8) | buffer[2]);
    lsm->accel_raw[2] = (int16_t)((buffer[5] << 8) | buffer[4]);
    
    // Convertir en m/s²
    lsm->acceleration.x = (lsm->accel_raw[0] * lsm->accel_mg_lsb / 1000.0f) * SENSORS_GRAVITY_STANDARD;
    lsm->acceleration.y = (lsm->accel_raw[1] * lsm->accel_mg_lsb / 1000.0f) * SENSORS_GRAVITY_STANDARD;
    lsm->acceleration.z = (lsm->accel_raw[2] * lsm->accel_mg_lsb / 1000.0f) * SENSORS_GRAVITY_STANDARD;
    
    // Lire le gyroscope
    if (i2c_read_block(lsm->fd_xg, LSM9DS1_REGISTER_OUT_X_L_G, buffer, 6) < 0) {
        return false;
    }
    lsm->gyro_raw[0] = (int16_t)((buffer[1] << 8) | buffer[0]);
    lsm->gyro_raw[1] = (int16_t)((buffer[3] << 8) | buffer[2]);
    lsm->gyro_raw[2] = (int16_t)((buffer[5] << 8) | buffer[4]);
    
    // Convertir en rad/s
    lsm->gyro.x = lsm->gyro_raw[0] * lsm->gyro_dps_digit * SENSORS_DPS_TO_RADS;
    lsm->gyro.y = lsm->gyro_raw[1] * lsm->gyro_dps_digit * SENSORS_DPS_TO_RADS;
    lsm->gyro.z = lsm->gyro_raw[2] * lsm->gyro_dps_digit * SENSORS_DPS_TO_RADS;
    
    // Lire le magnétomètre
    if (i2c_read_block(lsm->fd_mag, LIS3MDL_REGISTER_OUT_X_L, buffer, 6) < 0) {
        return false;
    }
    lsm->mag_raw[0] = (int16_t)((buffer[1] << 8) | buffer[0]);
    lsm->mag_raw[1] = (int16_t)((buffer[3] << 8) | buffer[2]);
    lsm->mag_raw[2] = (int16_t)((buffer[5] << 8) | buffer[4]);
    
    // Convertir en uT (microTesla)
    lsm->magnetic.x = lsm->mag_raw[0] * lsm->mag_gauss_lsb * 100.0f;
    lsm->magnetic.y = lsm->mag_raw[1] * lsm->mag_gauss_lsb * 100.0f;
    lsm->magnetic.z = lsm->mag_raw[2] * lsm->mag_gauss_lsb * 100.0f;
    
    // Lire la température
    uint8_t temp_buffer[2];
    if (i2c_read_block(lsm->fd_xg, LSM9DS1_REGISTER_TEMP_OUT_L, temp_buffer, 2) < 0) {
        return false;
    }
    lsm->temp_raw = (int16_t)((temp_buffer[1] << 8) | temp_buffer[0]);
    lsm->temperature = 21.0f + (float)lsm->temp_raw / 8.0f;
    
    return true;
}
