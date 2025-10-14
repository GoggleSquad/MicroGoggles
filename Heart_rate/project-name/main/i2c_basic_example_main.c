/*
 * Example: I2C communication with MAXREFDES117# (MAX30102)
 * Reads part ID and sample data registers over I2C.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

#define MAX_SAMPLES 500   // ~5 seconds @ 100 Hz
uint32_t ir_samples[MAX_SAMPLES];
int sample_idx = 0;

static const char *TAG = "maxrefdes117";

#define I2C_MASTER_SCL_IO           8
#define I2C_MASTER_SDA_IO           9
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000
#define I2C_MASTER_TIMEOUT_MS       1000

// MAX30102 I2C address
#define MAX30102_ADDR               0x57

// Some useful MAX30102 registers
#define REG_PART_ID                 0xFF
#define REG_INT_STATUS_1            0x00
#define REG_INT_STATUS_2            0x01
#define REG_FIFO_DATA               0x07


#define MAX_BRIGHTNESS 255
#define BUFFER_LENGTH 100

// Algorithm buffers
uint32_t ir_buffer[BUFFER_LENGTH];
uint32_t red_buffer[BUFFER_LENGTH];

// Algorithm results
int32_t spo2;
int8_t spo2_valid;
int32_t heart_rate;
int8_t hr_valid;

// Moving averages for smoother display
int32_t hr_avg = 0;
int32_t spo2_avg = 0;

#include "algorithm.h"


// Function to read from a register
static esp_err_t max30102_read_register(i2c_master_dev_handle_t dev_handle, uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev_handle, &reg, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

// Function to write to a register
static esp_err_t max30102_write_register(i2c_master_dev_handle_t dev_handle, uint8_t reg, uint8_t value)
{
    uint8_t write_buf[2] = {reg, value};
    return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

// I2C init
static void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MAX30102_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle));
}

// MAX30102 register addresses
#define REG_INTR_ENABLE_1      0x02
#define REG_INTR_ENABLE_2      0x03
#define REG_FIFO_WR_PTR        0x04
#define REG_OVF_COUNTER        0x05
#define REG_FIFO_RD_PTR        0x06
#define REG_FIFO_CONFIG        0x08
#define REG_MODE_CONFIG        0x09
#define REG_SPO2_CONFIG        0x0A
#define REG_LED1_PA            0x0C
#define REG_LED2_PA            0x0D
#define REG_MULTI_LED_CTRL1    0x11
#define REG_MULTI_LED_CTRL2    0x12

// Try these configuration changes first:

static void max30102_init(i2c_master_dev_handle_t dev_handle)
{
    // Reset device
    max30102_write_register(dev_handle, REG_MODE_CONFIG, 0x40);
    vTaskDelay(pdMS_TO_TICKS(300));

    // Enable interrupt on new FIFO data ready
    max30102_write_register(dev_handle, REG_INTR_ENABLE_1, 0xC0);
    max30102_write_register(dev_handle, REG_INTR_ENABLE_2, 0x00);

    // FIFO Configuration: NO averaging first, FIFO rollover, almost full = 17
    max30102_write_register(dev_handle, REG_FIFO_CONFIG, 0x0F); // Changed from 0x4F

    // Mode Configuration: SpO2 mode (RED + IR)
    max30102_write_register(dev_handle, REG_MODE_CONFIG, 0x03);

    // SPO2 Configuration: 100Hz, 411us pulse width, 18-bit resolution (max)
    max30102_write_register(dev_handle, REG_SPO2_CONFIG, 0x2F); // Changed to 18-bit

    // INCREASE LED power significantly - start with 6.4mA
    max30102_write_register(dev_handle, REG_LED1_PA, 0x1F); // RED - lower
    max30102_write_register(dev_handle, REG_LED2_PA, 0x1F); // IR - increase this!

    ESP_LOGI(TAG, "MAX30102 configured for SpO2 mode");
}

// Better filtering and AC/DC extraction
typedef struct {
    int32_t dc_filter_w;
    int32_t dc_filter_result;
    int16_t lpb_filter_ir[2];
    int32_t last_dc_estimate;
} filter_state_t;

static filter_state_t filter_state = {0};

// DC removal filter
int32_t dc_removal(int32_t x, int32_t *w, int32_t *result)
{
    *w = x + (((*w) * 95) >> 7); // Simplified: multiply by 0.95
    *result = *w - ((*w) >> 7);
    return x - *result;
}

// Simple low-pass butterworth filter for smoothing
int16_t low_pass_filter(int32_t x, int16_t *v)
{
    v[0] = v[1];
    // Butterworth coefficients for ~3Hz cutoff at 100Hz sample rate
    int32_t tmp = (((x * 2048) - (v[0] * 3686)) >> 11);
    v[1] = (int16_t)tmp;
    return (v[0] + v[1]);
}

#define BUFFER_SIZE 100
int32_t ir_ac_buffer[BUFFER_SIZE];
int buffer_idx = 0;

void app_main(void)
{
    uint8_t data[6];
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;

    i2c_master_init(&bus_handle, &dev_handle);
    ESP_LOGI(TAG, "I2C initialized successfully");

    uint8_t part_id = 0;
    ESP_ERROR_CHECK(max30102_read_register(dev_handle, REG_PART_ID, &part_id, 1));
    ESP_LOGI(TAG, "MAX30102 Part ID: 0x%02X", part_id);

    max30102_init(dev_handle);

    int samples_collected = 0;
    int32_t dc_estimate = 0;
    
    while (1) {
    esp_err_t ret = max30102_read_register(dev_handle, REG_FIFO_DATA, data, 6);
    if (ret == ESP_OK) {
        // Shift buffer
        for (int i = 0; i < BUFFER_LENGTH - 1; i++) {
            red_buffer[i] = red_buffer[i + 1];
            ir_buffer[i] = ir_buffer[i + 1];
        }

        red_buffer[BUFFER_LENGTH - 1] = ((uint32_t)data[0] << 16) |
                                        ((uint32_t)data[1] << 8) | data[2];
        ir_buffer[BUFFER_LENGTH - 1]  = ((uint32_t)data[3] << 16) |
                                        ((uint32_t)data[4] << 8) | data[5];
        red_buffer[BUFFER_LENGTH - 1] &= 0x3FFFF;
        ir_buffer[BUFFER_LENGTH - 1] &= 0x3FFFF;

        sample_idx++;

        if (sample_idx >= 25) { // run every 25 samples (~0.25 s)
            maxim_heart_rate_and_oxygen_saturation(
                ir_buffer,
                BUFFER_LENGTH,
                red_buffer,
                &spo2,
                &spo2_valid,
                &heart_rate,
                &hr_valid
            );

            if (hr_valid)  hr_avg  = (hr_avg  == 0) ? heart_rate : (hr_avg  * 3 + heart_rate) / 4;
            if (spo2_valid) spo2_avg = (spo2_avg == 0) ? spo2       : (spo2_avg * 3 + spo2) / 4;

            ESP_LOGI(TAG, "HR: %ld BPM | SpO2: %ld%% | IR: %lu",
                    hr_avg, spo2_avg, ir_buffer[BUFFER_LENGTH - 1]);

            sample_idx = 0;
        }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
    }
}












// // // /*
// // //  * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
// // //  *
// // //  * SPDX-License-Identifier: Unlicense OR CC0-1.0
// // //  */
// // // /* i2c - Simple Example

// // //    Simple I2C example that shows how to initialize I2C
// // //    as well as reading and writing from and to registers for a sensor connected over I2C.

// // //    The sensor used in this example is a MPU9250 inertial measurement unit.
// // // */


// // // #include "freertos/FreeRTOS.h"
// // // #include "freertos/task.h"
// // // #include "driver/i2c_master.h"
// // // #include "esp_log.h"

// // // #define I2C_MASTER_SCL_IO  8   // Change to your SCL pin
// // // #define I2C_MASTER_SDA_IO  9   // Change to your SDA pin
// // // #define I2C_MASTER_NUM     I2C_NUM_0
// // // #define I2C_MASTER_FREQ_HZ 100000

// // // static const char *TAG = "i2c_scan";

// // // void app_main(void)
// // // {
// // //     i2c_config_t conf = {
// // //         .mode = I2C_MODE_MASTER,
// // //         .sda_io_num = I2C_MASTER_SDA_IO,
// // //         .scl_io_num = I2C_MASTER_SCL_IO,
// // //         .sda_pullup_en = GPIO_PULLUP_ENABLE,
// // //         .scl_pullup_en = GPIO_PULLUP_ENABLE,
// // //         .master.clk_speed = I2C_MASTER_FREQ_HZ
// // //     };

// // //     esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
// // //     if (ret != ESP_OK) {
// // //         ESP_LOGE(TAG, "I2C param config failed");
// // //         return;
// // //     }

// // //     ret = i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0);
// // //     if (ret != ESP_OK) {
// // //         ESP_LOGE(TAG, "I2C driver install failed");
// // //         return;
// // //     }

// // //     ESP_LOGI(TAG, "Scanning I2C bus...");

// // //     for (uint8_t addr = 1; addr < 127; addr++) {
// // //         i2c_cmd_handle_t cmd = i2c_cmd_link_create();
// // //         i2c_master_start(cmd);
// // //         i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
// // //         i2c_master_stop(cmd);
// // //         ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
// // //         i2c_cmd_link_delete(cmd);

// // //         if (ret == ESP_OK) {
// // //             ESP_LOGI(TAG, "Found device at 0x%02X", addr);
// // //         }
// // //     }

// // //     ESP_LOGI(TAG, "I2C scan done.");
// // // }


// // // /*
// // // #include <stdio.h>
// // // #include "sdkconfig.h"
// // // #include "freertos/FreeRTOS.h"
// // // #include "freertos/task.h"
// // // #include "esp_log.h"
// // // #include "driver/i2c_master.h"

// // // // static const char *TAG = "example";
// // // static const char *TAG = "heart_i2c";

// // // #define I2C_MASTER_SCL_IO           8 //CONFIG_I2C_MASTER_SCL       /*!< GPIO number used for I2C master clock */
// // // #define I2C_MASTER_SDA_IO           9 //CONFIG_I2C_MASTER_SDA       /*!< GPIO number used for I2C master data  */
// // // #define I2C_MASTER_NUM              I2C_NUM_0                   /*!< I2C port number for master dev */
// // // #define I2C_MASTER_FREQ_HZ          CONFIG_I2C_MASTER_FREQUENCY /*!< I2C master clock frequency */
// // // #define I2C_MASTER_TX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
// // // #define I2C_MASTER_RX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
// // // #define I2C_MASTER_TIMEOUT_MS       1000*/

// // // /*
// // // // #define MPU9250_SENSOR_ADDR         0x68        /*!< Address of the MPU9250 sensor */
// // // // #define MPU9250_WHO_AM_I_REG_ADDR   0x75        /*!< Register addresses of the "who am I" register */
// // // // #define MPU9250_PWR_MGMT_1_REG_ADDR 0x6B        /*!< Register addresses of the power management register */
// // // // #define MPU9250_RESET_BIT           7
// // // // Heart rate sensor configuration
// // // #define HEART_SENSOR_ADDR           0x57  // Replace with your sensor's I2C address
// // // #define HEART_SENSOR_DATA_REG       0x00  // Replace with the correct data register

// // // /**
// // //  * @brief Read bytes from the heart rate sensor
// // //  */
// // // static esp_err_t heart_sensor_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len)
// // // {
// // //     return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
// // // }

// // // /**
// // //  * @brief I2C master initialization
// // //  */
// // // static void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle)
// // // {
// // //     i2c_master_bus_config_t bus_config = {
// // //         .i2c_port = I2C_MASTER_NUM,
// // //         .sda_io_num = I2C_MASTER_SDA_IO,
// // //         .scl_io_num = I2C_MASTER_SCL_IO,
// // //         .clk_source = I2C_CLK_SRC_DEFAULT,
// // //         .glitch_ignore_cnt = 7,
// // //         .flags.enable_internal_pullup = true,
// // //     };

// // //     esp_err_t ret = i2c_new_master_bus(&bus_config, bus_handle);
// // //     if (ret != ESP_OK) {
// // //         ESP_LOGE(TAG, "Failed to create I2C bus: %d", ret);
// // //         return;
// // //     }

// // //     i2c_device_config_t dev_config = {
// // //         .dev_addr_length = I2C_ADDR_BIT_LEN_7,
// // //         .device_address = HEART_SENSOR_ADDR,
// // //         .scl_speed_hz = I2C_MASTER_FREQ_HZ,
// // //     };

// // //     ret = i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle);
// // //     if (ret != ESP_OK) {
// // //         ESP_LOGE(TAG, "Failed to add I2C device: %d", ret);
// // //         return;
// // //     }

// // //     ESP_LOGI(TAG, "I2C bus and device initialized successfully");
// // // }

// // // void app_main(void)
// // // {
// // //     uint8_t data[2];
// // //     i2c_master_bus_handle_t bus_handle;
// // //     i2c_master_dev_handle_t dev_handle;

// // //     i2c_master_init(&bus_handle, &dev_handle);

// // //     while (1) {
// // //         // Read 2 bytes from the heart sensor
// // //         esp_err_t ret = heart_sensor_read(dev_handle, HEART_SENSOR_DATA_REG, data, 2);
// // //         if (ret != ESP_OK) {
// // //             ESP_LOGE(TAG, "Heart sensor read failed: %d", ret);
// // //         } else {
// // //             uint16_t heart_value = (data[0] << 8) | data[1];
// // //             ESP_LOGI(TAG, "Heart sensor value = %d", heart_value);
// // //         }

// // //         vTaskDelay(pdMS_TO_TICKS(1000)); // 1 Hz read rate
// // //     }

// // //     // Optional de-init (never reached in this loop)
// // //     // ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));
// // //     // ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
// // //     // ESP_LOGI(TAG, "I2C de-initialized successfully");
// // // }







// // //old template stuff below


// //  * @brief Read a sequence of bytes from a MPU9250 sensor registers
// //  */
// // static esp_err_t mpu9250_register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len)
// // {
// //     return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
// // }

// // /**
// //  * @brief Write a byte to a MPU9250 sensor register
// //  */
// // static esp_err_t mpu9250_register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data)
// // {
// //     uint8_t write_buf[2] = {reg_addr, data};
// //     return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
// // }

// // /**
// //  * @brief i2c master initialization
// //  */
// // static void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle)
// // {
// //     i2c_master_bus_config_t bus_config = {
// //         .i2c_port = I2C_MASTER_NUM,
// //         .sda_io_num = I2C_MASTER_SDA_IO,
// //         .scl_io_num = I2C_MASTER_SCL_IO,
// //         .clk_source = I2C_CLK_SRC_DEFAULT,
// //         .glitch_ignore_cnt = 7,
// //         .flags.enable_internal_pullup = true,
// //     };
// //     ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));

// //     i2c_device_config_t dev_config = {
// //         .dev_addr_length = I2C_ADDR_BIT_LEN_7,
// //         .device_address = MPU9250_SENSOR_ADDR,
// //         .scl_speed_hz = I2C_MASTER_FREQ_HZ,
// //     };
// //     ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle));
// // }

// // void app_main(void)
// // {
// //     uint8_t data[2];
// //     i2c_master_bus_handle_t bus_handle;
// //     i2c_master_dev_handle_t dev_handle;
// //     i2c_master_init(&bus_handle, &dev_handle);
// //     ESP_LOGI(TAG, "I2C initialized successfully");

// //     /* Read the MPU9250 WHO_AM_I register, on power up the register should have the value 0x71 */
// //     ESP_ERROR_CHECK(mpu9250_register_read(dev_handle, MPU9250_WHO_AM_I_REG_ADDR, data, 1));
// //     ESP_LOGI(TAG, "WHO_AM_I = %X", data[0]);

// //     /* Demonstrate writing by resetting the MPU9250 */
// //     ESP_ERROR_CHECK(mpu9250_register_write_byte(dev_handle, MPU9250_PWR_MGMT_1_REG_ADDR, 1 << MPU9250_RESET_BIT));

// //     ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));
// //     ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
// //     ESP_LOGI(TAG, "I2C de-initialized successfully");
// // }

// /*
//  * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
//  *
//  * SPDX-License-Identifier: Unlicense OR CC0-1.0
//  */

// /* i2c - Simple Example

//    Simple I2C example that shows how to initialize I2C
//    as well as reading and writing from and to registers for a sensor connected over I2C.

//    The sensor used in this example is a MPU9250 inertial measurement unit.
// */

// #include <stdio.h>
// #include "sdkconfig.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_log.h"
// #include "driver/i2c_master.h"

// static const char *TAG = "example";

// #define I2C_MASTER_SCL_IO           8       /*!< GPIO number used for I2C master clock */
// #define I2C_MASTER_SDA_IO           9       /*!< GPIO number used for I2C master data  */
// #define I2C_MASTER_NUM              I2C_NUM_0                   /*!< I2C port number for master dev */
// #define I2C_MASTER_FREQ_HZ          CONFIG_I2C_MASTER_FREQUENCY /*!< I2C master clock frequency */
// #define I2C_MASTER_TX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
// #define I2C_MASTER_RX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
// #define I2C_MASTER_TIMEOUT_MS       1000

// #define MPU9250_SENSOR_ADDR         0x57        /*!< Address of the MPU9250 sensor */
// #define MPU9250_WHO_AM_I_REG_ADDR   0x75        /*!< Register addresses of the "who am I" register */
// #define MPU9250_PWR_MGMT_1_REG_ADDR 0x6B        /*!< Register addresses of the power management register */
// #define MPU9250_RESET_BIT           7

// /**
//  * @brief Read a sequence of bytes from a MPU9250 sensor registers
//  */
// static esp_err_t mpu9250_register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len)
// {
//     return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
// }

// /**
//  * @brief Write a byte to a MPU9250 sensor register
//  */
// static esp_err_t mpu9250_register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data)
// {
//     uint8_t write_buf[2] = {reg_addr, data};
//     return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
// }

// /**
//  * @brief i2c master initialization
//  */
// static void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle)
// {
//     i2c_master_bus_config_t bus_config = {
//         .i2c_port = I2C_MASTER_NUM,
//         .sda_io_num = I2C_MASTER_SDA_IO,
//         .scl_io_num = I2C_MASTER_SCL_IO,
//         .clk_source = I2C_CLK_SRC_DEFAULT,
//         .glitch_ignore_cnt = 7,
//         .flags.enable_internal_pullup = true,
//     };
//     ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));

//     i2c_device_config_t dev_config = {
//         .dev_addr_length = I2C_ADDR_BIT_LEN_7,
//         .device_address = MPU9250_SENSOR_ADDR,
//         .scl_speed_hz = I2C_MASTER_FREQ_HZ,
//     };
//     ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle));
// }

// void app_main(void)
// {
//     uint8_t data[2];
//     i2c_master_bus_handle_t bus_handle;
//     i2c_master_dev_handle_t dev_handle;
//     i2c_master_init(&bus_handle, &dev_handle);
//     ESP_LOGI(TAG, "I2C initialized successfully");

//     /* Read the MPU9250 WHO_AM_I register, on power up the register should have the value 0x71 */
//     ESP_ERROR_CHECK(mpu9250_register_read(dev_handle, MPU9250_WHO_AM_I_REG_ADDR, data, 1));
//     ESP_LOGI(TAG, "WHO_AM_I = %X", data[0]);

//     /* Demonstrate writing by resetting the MPU9250 */
//     ESP_ERROR_CHECK(mpu9250_register_write_byte(dev_handle, MPU9250_PWR_MGMT_1_REG_ADDR, 1 << MPU9250_RESET_BIT));

//     ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));
//     ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
//     ESP_LOGI(TAG, "I2C de-initialized successfully");
// }
