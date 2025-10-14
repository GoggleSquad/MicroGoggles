#pragma once

#include "esp_err.h"
#include "driver/i2c.h"

// Initialize LIS3DH on specified I2C port
esp_err_t lis3dh_init(i2c_port_t i2c_num);

// Read WHO_AM_I register
uint8_t lis3dh_who_am_i(i2c_port_t i2c_num);

// Read raw accelerometer data
esp_err_t lis3dh_read_accel(i2c_port_t i2c_num, int16_t *x, int16_t *y, int16_t *z);

// Convert raw accelerometer values to g
void lis3dh_convert_to_g(int16_t raw_x, int16_t raw_y, int16_t raw_z,
                         float *gx, float *gy, float *gz);
