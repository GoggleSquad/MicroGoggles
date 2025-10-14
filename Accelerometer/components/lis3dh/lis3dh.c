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
    uint8_t ctrl1 = 0x57; // 100Hz, all axes enabled
    uint8_t ctrl4 = 0x00; // ±2g range

    // Write CTRL_REG1
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DEV_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, LIS3DH_REG_CTRL1, true);
    i2c_master_write_byte(cmd, ctrl1, true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) return ret;

    // Write CTRL_REG4
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DEV_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, LIS3DH_REG_CTRL4, true);
    i2c_master_write_byte(cmd, ctrl4, true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) return ret;

    ESP_LOGI(TAG, "LIS3DH initialized (100Hz, ±2g)");
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
    const float g = 9.81F; // 1g in m/s²
    const float scale_factor = 0.001F; // ≈ (2g / 2048)
    *gx = raw_x * scale_factor * g;
    *gy = raw_y * scale_factor * g;
    *gz = raw_z * scale_factor * g;
}
