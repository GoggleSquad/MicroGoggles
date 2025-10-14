#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "lis3dh.h"

#define I2C_MASTER_SCL_IO           9
#define I2C_MASTER_SDA_IO           8
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0

static const char *TAG = "MAIN_APP";

// Initialize I2C master
static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) return ret;
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode,
                              I2C_MASTER_RX_BUF_DISABLE,
                              I2C_MASTER_TX_BUF_DISABLE, 0);
}

// Scan the I2C bus for devices
static void i2c_scan_bus()
{
    ESP_LOGI(TAG, "Scanning I2C bus...");
    int found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        if (i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 100 / portTICK_PERIOD_MS) == ESP_OK) {
            ESP_LOGI(TAG, "Found device at 0x%02X", addr);
            found++;
        }
        i2c_cmd_link_delete(cmd);
    }
    if (found == 0) {
        ESP_LOGW(TAG, "No I2C devices found!");
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Initializing I2C...");
    if (i2c_master_init() != ESP_OK) {
        ESP_LOGE(TAG, "I2C master init failed!");
        return;
    }

    i2c_scan_bus(); // Scan bus before initializing sensor

    // Read WHO_AM_I
    uint8_t whoami = lis3dh_who_am_i(I2C_MASTER_NUM);
    ESP_LOGI(TAG, "LIS3DH WHO_AM_I = 0x%02X", whoami);

    // Check if the value matches expected LIS3DH response
    if (whoami == 0x33) {
        ESP_LOGI(TAG, "LIS3DH detected, initializing...");
        if (lis3dh_init(I2C_MASTER_NUM) == ESP_OK) {
            ESP_LOGI(TAG, "LIS3DH initialized successfully");

            int16_t x, y, z;
            float gx, gy, gz;

            while (1) {
                if (lis3dh_read_accel(I2C_MASTER_NUM, &x, &y, &z) == ESP_OK) {
                    lis3dh_convert_to_g(x, y, z, &gx, &gy, &gz);
                    ESP_LOGI(TAG, "Accel: X=%.3f g, Y=%.3f g, Z=%.3f g", gx, gy, gz);
                } else {
                    ESP_LOGE(TAG, "Failed to read acceleration");
                }
                vTaskDelay(pdMS_TO_TICKS(500));
            }

        } else {
            ESP_LOGE(TAG, "Failed to initialize LIS3DH");
        }
    } else {
        ESP_LOGW(TAG, "LIS3DH not detected! WHO_AM_I returned 0x%02X", whoami);
    }
}





// #include <stdio.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "driver/i2c.h"
// #include "esp_log.h"
// #include "lis3dh.h"

// #define I2C_MASTER_SCL_IO           9
// #define I2C_MASTER_SDA_IO           8
// #define I2C_MASTER_NUM              I2C_NUM_0
// #define I2C_MASTER_FREQ_HZ          100000
// #define I2C_MASTER_TX_BUF_DISABLE   0
// #define I2C_MASTER_RX_BUF_DISABLE   0

// static const char *TAG = "MAIN_APP";

// static esp_err_t i2c_master_init(void)
// {
//     i2c_config_t conf = {
//         .mode = I2C_MODE_MASTER,
//         .sda_io_num = 8,
//         .scl_io_num = 9,
//         .sda_pullup_en = GPIO_PULLUP_ENABLE,
//         .scl_pullup_en = GPIO_PULLUP_ENABLE,
//         .master.clk_speed = I2C_MASTER_FREQ_HZ,
//     };
//     esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
//     if (ret != ESP_OK) return ret;
//     return i2c_driver_install(I2C_MASTER_NUM, conf.mode,
//                               I2C_MASTER_RX_BUF_DISABLE,
//                               I2C_MASTER_TX_BUF_DISABLE, 0);
// }

// void app_main(void)
// {
//     ESP_LOGI(TAG, "Initializing I2C...");
//     ESP_ERROR_CHECK(i2c_master_init());

//     ESP_LOGI(TAG, "Initializing LIS3DH...");
//     ESP_ERROR_CHECK(lis3dh_init(I2C_MASTER_NUM));

//     uint8_t whoami = lis3dh_who_am_i(I2C_MASTER_NUM);
//     ESP_LOGI(TAG, "LIS3DH WHO_AM_I = 0x%02X", whoami);

//     int16_t x, y, z;
//     float gx, gy, gz;

//     while (1) {
//         if (lis3dh_read_accel(I2C_MASTER_NUM, &x, &y, &z) == ESP_OK) {
//             lis3dh_convert_to_g(x, y, z, &gx, &gy, &gz);
//             ESP_LOGI(TAG, "Accel: X=%.3f g, Y=%.3f g, Z=%.3f g", gx, gy, gz);
//         } else {
//             ESP_LOGE(TAG, "Failed to read acceleration");
//         }
//         vTaskDelay(pdMS_TO_TICKS(500));
//     }
// }
