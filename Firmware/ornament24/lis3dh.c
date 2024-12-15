#include "lis3dh.h"
#include <math.h>



// Previous angle values for filtering
static float prev_pitch = 0.0f;
static float prev_roll = 0.0f;

// Initialize the LIS3DH
bool lis3dh_init(void) 
{
    // Initialize I2C interface
    i2c_init(I2C_PORT, I2C_FREQ);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    // Check device ID
    uint8_t who_am_i = lis3dh_read_reg(REG_WHO_AM_I);
    if (who_am_i != 0x33) {  // 0x33 is the expected ID for LIS3DH
        printf("Error: Wrong device ID (0x%02X)\n", who_am_i);
        return false;
    }

    // Configure the device:
    // CTRL_REG1: Enable all axes, normal mode, 100Hz data rate
    lis3dh_write_reg(REG_CTRL_REG1, 0x57);  // 0b01010111

    // CTRL_REG4: Full-scale = ±2g, high-resolution mode
    lis3dh_write_reg(REG_CTRL_REG4, 0x08);  // 0b00001000

    return true;
}

// Read a register from the LIS3DH
uint8_t lis3dh_read_reg(uint8_t reg) 
{
    uint8_t data;
    i2c_write_blocking(I2C_PORT, LIS3DH_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, LIS3DH_ADDR, &data, 1, false);
    return data;
}

// Write to a register on the LIS3DH
void lis3dh_write_reg(uint8_t reg, uint8_t value) 
{
    uint8_t data[2] = {reg, value};
    i2c_write_blocking(I2C_PORT, LIS3DH_ADDR, data, 2, false);
}

// Read acceleration data from all three axes
acceleration_t lis3dh_read_acceleration(void) 
{
    acceleration_t accel;
    uint8_t data[6];
    uint8_t reg = REG_OUT_X_L | 0x80;  // Set MSB to enable auto-increment

    i2c_write_blocking(I2C_PORT, LIS3DH_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, LIS3DH_ADDR, data, 6, false);

    // Combine high and low bytes and convert to 16-bit signed integers
    accel.x = (int16_t)((data[1] << 8) | data[0]);
    accel.y = (int16_t)((data[3] << 8) | data[2]);
    accel.z = (int16_t)((data[5] << 8) | data[4]);

    return accel;
}

// Apply low-pass filter to smooth angle readings
void apply_low_pass_filter(float *prev_value, float new_value, float alpha) 
{
    *prev_value = (*prev_value * alpha) + (new_value * (1.0f - alpha));
}


// Calculate pitch and roll angles from acceleration data
angle_t calculate_angles(acceleration_t accel) 
{
    angle_t angles;
    float x_g = accel.x * 2.0f / 32768.0f;
    float y_g = accel.y * 2.0f / 32768.0f;
    float z_g = accel.z * 2.0f / 32768.0f;
    
    // Calculate pitch (rotation around X-axis)
    float pitch = atan2f(x_g, sqrtf(y_g * y_g + z_g * z_g)) * RAD_TO_DEG;
    
    // Calculate roll (rotation around Y-axis)
    float roll = atan2f(y_g, sqrtf(x_g * x_g + z_g * z_g)) * RAD_TO_DEG;
    
    // Apply low-pass filter (alpha = 0.8 means 80% of old value, 20% of new value)
    apply_low_pass_filter(&prev_pitch, pitch, 0.8f);
    apply_low_pass_filter(&prev_roll, roll, 0.8f);
    
    angles.pitch = prev_pitch;
    angles.roll = prev_roll;
    
    return angles;
}


