#include "lis3dh.h"
#include "esp_log.h"
#include "driver/i2c.h"

static const char *TAG = "lis3dh";

#define LIS3DH_REG_WHO_AM_I   0x0F
#define LIS3DH_REG_CTRL1      0x20
#define LIS3DH_REG_CTRL4      0x23
#define LIS3DH_REG_OUT_X_L    0x28

#define LIS3DH_WHO_AM_I_VAL   0x33
#define DEV_ADDR              0x19  // LIS3DH default I2C address

esp_err_t lis3dh_init(i2c_port_t i2c_num)
{
    esp_err_t ret;

    // Control Register 1: ODR = 400Hz, Enable X, Y, Z
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DEV_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, LIS3DH_REG_CTRL1, true);
    i2c_master_write_byte(cmd, 0x77, true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write CTRL1 (err %d)", ret);
        return ret;
    }

    // Control Register 4: +/-2g, default settings
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DEV_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, LIS3DH_REG_CTRL4, true);
    i2c_master_write_byte(cmd, 0x00, true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write CTRL4 (err %d)", ret);
        return ret;
    }

    return ESP_OK;
}

uint8_t lis3dh_who_am_i(i2c_port_t i2c_num)
{
    esp_err_t ret;
    uint8_t data = 0;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DEV_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, LIS3DH_REG_WHO_AM_I, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DEV_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &data, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I (err %d)", ret);
        return 0;
    }
    return data;
}

esp_err_t lis3dh_read_accel(i2c_port_t i2c_num, int16_t *x, int16_t *y, int16_t *z)
{
    esp_err_t ret;
    uint8_t data[6];
    uint8_t reg = LIS3DH_REG_OUT_X_L | 0x80; // auto-increment

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DEV_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DEV_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 6, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) return ret;

    *x = (int16_t)((data[1] << 8) | data[0]) >> 4;
    *y = (int16_t)((data[3] << 8) | data[2]) >> 4;
    *z = (int16_t)((data[5] << 8) | data[4]) >> 4;

    ESP_LOGI(TAG, "Accel raw X=%d Y=%d Z=%d", *x, *y, *z);
    return ESP_OK;
}

void lis3dh_convert_to_g(int16_t raw_x, int16_t raw_y, int16_t raw_z, float *gx, float *gy, float *gz)
{
    // LIS3DH ±2g mode: 12-bit resolution, 2048 LSB/g
    // Sensitivity = 2g / 2048 = 0.0009765625 g/LSB
    // Convert to m/s²: multiply by 9.80665
    const float sensitivity = 2.0f / 2048.0f; // g per LSB
    const float g_to_ms2 = 9.80665f; // 1g in m/s²

    *gx = (float)raw_x * sensitivity * g_to_ms2;
    *gy = (float)raw_y * sensitivity * g_to_ms2;
    *gz = (float)raw_z * sensitivity * g_to_ms2;

    ESP_LOGI(TAG, "Converted - X: %.3f, Y: %.3f, Z: %.3f m/s²", *gx, *gy, *gz);
}
