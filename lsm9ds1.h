#ifndef LSM9DS1_H
#define LSM9DS1_H

#include <stdint.h>
#include <stdbool.h>

// Adresses I2C
#define LSM9DS1_ADDRESS_ACCELGYRO  0x6B
#define LSM9DS1_ADDRESS_MAG        0x1E
#define LSM9DS1_XG_ID              0x68

// Registres Accel/Gyro
#define LSM9DS1_REGISTER_WHO_AM_I_XG    0x0F
#define LSM9DS1_REGISTER_CTRL_REG1_G    0x10
#define LSM9DS1_REGISTER_CTRL_REG5_XL   0x1F
#define LSM9DS1_REGISTER_CTRL_REG6_XL   0x20
#define LSM9DS1_REGISTER_CTRL_REG8      0x22
#define LSM9DS1_REGISTER_OUT_X_L_G      0x18
#define LSM9DS1_REGISTER_OUT_X_L_XL     0x28
#define LSM9DS1_REGISTER_TEMP_OUT_L     0x15

// Registres Mag
#define LIS3MDL_REGISTER_WHO_AM_I       0x0F
#define LIS3MDL_REGISTER_CTRL_REG1      0x20
#define LIS3MDL_REGISTER_CTRL_REG2      0x21
#define LIS3MDL_REGISTER_CTRL_REG3      0x22
#define LIS3MDL_REGISTER_OUT_X_L        0x28
#define LIS3MDL_ID                      0x3D

// Plages de l'accéléromètre
typedef enum {
    LSM9DS1_ACCELRANGE_2G  = (0b00 << 3),
    LSM9DS1_ACCELRANGE_4G  = (0b10 << 3),
    LSM9DS1_ACCELRANGE_8G  = (0b11 << 3),
    LSM9DS1_ACCELRANGE_16G = (0b01 << 3)
} lsm9ds1_accel_range_t;

// Fréquences de données de l'accéléromètre
typedef enum {
    LSM9DS1_ACCELDATARATE_POWERDOWN = (0b0000 << 5),
    LSM9DS1_ACCELDATARATE_10HZ      = (0b001 << 5),
    LSM9DS1_ACCELDATARATE_50HZ      = (0b010 << 5),
    LSM9DS1_ACCELDATARATE_119HZ     = (0b011 << 5),
    LSM9DS1_ACCELDATARATE_238HZ     = (0b100 << 5),
    LSM9DS1_ACCELDATARATE_476HZ     = (0b101 << 5),
    LSM9DS1_ACCELDATARATE_952HZ     = (0b110 << 5)
} lsm9ds1_accel_datarate_t;

// Échelles du gyroscope
typedef enum {
    LSM9DS1_GYROSCALE_245DPS  = (0b00 << 3),
    LSM9DS1_GYROSCALE_500DPS  = (0b01 << 3),
    LSM9DS1_GYROSCALE_2000DPS = (0b11 << 3)
} lsm9ds1_gyro_scale_t;

// Gains du magnétomètre
typedef enum {
    LSM9DS1_MAGGAIN_4GAUSS  = 0,
    LSM9DS1_MAGGAIN_8GAUSS  = 1,
    LSM9DS1_MAGGAIN_12GAUSS = 2,
    LSM9DS1_MAGGAIN_16GAUSS = 3
} lsm9ds1_mag_gain_t;

// Structure pour les données 3 axes
typedef struct {
    float x;
    float y;
    float z;
} vector3_t;

// Structure principale du capteur
typedef struct {
    int fd_xg;          // File descriptor pour accel/gyro
    int fd_mag;         // File descriptor pour magnétomètre
    float accel_mg_lsb;
    float gyro_dps_digit;
    float mag_gauss_lsb;
    
    // Données brutes
    int16_t accel_raw[3];
    int16_t gyro_raw[3];
    int16_t mag_raw[3];
    int16_t temp_raw;
    
    // Données converties
    vector3_t acceleration;  // m/s²
    vector3_t gyro;         // rad/s
    vector3_t magnetic;     // uT
    float temperature;      // °C
} lsm9ds1_t;

// Fonctions principales
bool lsm9ds1_init(lsm9ds1_t *lsm, const char *i2c_bus);
void lsm9ds1_close(lsm9ds1_t *lsm);
bool lsm9ds1_read(lsm9ds1_t *lsm);
void lsm9ds1_setup_accel(lsm9ds1_t *lsm, lsm9ds1_accel_range_t range, lsm9ds1_accel_datarate_t rate);
void lsm9ds1_setup_gyro(lsm9ds1_t *lsm, lsm9ds1_gyro_scale_t scale);
void lsm9ds1_setup_mag(lsm9ds1_t *lsm, lsm9ds1_mag_gain_t gain);

#endif // LSM9DS1_H

