#ifndef LIS3DH_H
#define LIS3DH_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"

// LIS3DH Register Addresses
#define LIS3DH_ADDR            0x19    // I2C Address (SA0 left floating)
#define REG_WHO_AM_I           0x0F
#define REG_CTRL_REG1          0x20
#define REG_CTRL_REG4          0x23
#define REG_OUT_X_L            0x28
#define REG_OUT_X_H            0x29
#define REG_OUT_Y_L            0x2A
#define REG_OUT_Y_H            0x2B
#define REG_OUT_Z_L            0x2C
#define REG_OUT_Z_H            0x2D

// I2C Configuration
#define I2C_PORT        i2c0
#define I2C_SDA_PIN     12
#define I2C_SCL_PIN     13
#define I2C_FREQ        400000  // 400 kHz

// Math constants
#define RAD_TO_DEG     57.295779513f    // 180/pi

typedef struct 
{
    int16_t x;
    int16_t y;
    int16_t z;
} acceleration_t;

typedef struct 
{
    float pitch;    // Rotation around X-axis (forward/backward tilt)
    float roll;     // Rotation around Y-axis (left/right tilt)
} angle_t;

// Function prototypes
bool lis3dh_init(void);
uint8_t lis3dh_read_reg(uint8_t reg);
void lis3dh_write_reg(uint8_t reg, uint8_t value);
acceleration_t lis3dh_read_acceleration(void);
angle_t calculate_angles(acceleration_t accel);
void apply_low_pass_filter(float *prev_value, float new_value, float alpha);

#endif