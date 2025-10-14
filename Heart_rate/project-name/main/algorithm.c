// /*******************************************************************************
// * Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
// * Modified for ESP-IDF compatibility
// *******************************************************************************/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

static const char *TAG = "maxrefdes117";

#define I2C_MASTER_SCL_IO           8
#define I2C_MASTER_SDA_IO           9
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000
#define I2C_MASTER_TIMEOUT_MS       1000

#define MAX30102_ADDR               0x57
#define REG_PART_ID                 0xFF
#define REG_FIFO_DATA               0x07

// Maxim algorithm configuration
#define MAX_BRIGHTNESS              255
#define BUFFER_LENGTH               100  // Must match algorithm requirements

// These buffers are required by Maxim's algorithm
uint32_t ir_buffer[BUFFER_LENGTH];
uint32_t red_buffer[BUFFER_LENGTH];

// Include the algorithm header
#include "algorithm.h"

// Algorithm outputs
int32_t spo2;
int8_t spo2_valid;
int32_t heart_rate;
int8_t hr_valid;

// Your existing I2C functions
static esp_err_t max30102_read_register(i2c_master_dev_handle_t dev_handle, 
                                       uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev_handle, &reg, 1, data, len, 
                                      I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t max30102_write_register(i2c_master_dev_handle_t dev_handle, 
                                         uint8_t reg, uint8_t value)
{
    uint8_t write_buf[2] = {reg, value};
    return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), 
                              I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static void i2c_master_init(i2c_master_bus_handle_t *bus_handle, 
                           i2c_master_dev_handle_t *dev_handle)
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

// Register definitions
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

static void max30102_init(i2c_master_dev_handle_t dev_handle)
{
    // Reset device
    max30102_write_register(dev_handle, REG_MODE_CONFIG, 0x40);
    vTaskDelay(pdMS_TO_TICKS(100));

    // FIFO Configuration: sample average = 4, rollover enabled
    max30102_write_register(dev_handle, REG_FIFO_CONFIG, 0x4F);

    // Mode Configuration: SpO2 mode (RED + IR)
    max30102_write_register(dev_handle, REG_MODE_CONFIG, 0x03);

    // SPO2 Configuration: 100Hz, 411us pulse width, 18-bit resolution
    max30102_write_register(dev_handle, REG_SPO2_CONFIG, 0x2F);

    // LED pulse amplitudes - adjust these based on your readings
    // Start low, increase if signal is weak
    max30102_write_register(dev_handle, REG_LED1_PA, 0x24); // RED
    max30102_write_register(dev_handle, REG_LED2_PA, 0x24); // IR

    // Enable data ready interrupt
    max30102_write_register(dev_handle, REG_INTR_ENABLE_1, 0xC0);
    max30102_write_register(dev_handle, REG_INTR_ENABLE_2, 0x00);

    ESP_LOGI(TAG, "MAX30102 initialized");
}

void app_main(void)
{
    uint8_t data[6];
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;

    i2c_master_init(&bus_handle, &dev_handle);
    ESP_LOGI(TAG, "I2C initialized successfully");

    uint8_t part_id = 0;
    ESP_ERROR_CHECK(max30102_read_register(dev_handle, REG_PART_ID, &part_id, 1));
    ESP_LOGI(TAG, "MAX30102 Part ID: 0x%02X (should be 0x15)", part_id);

    max30102_init(dev_handle);

    ESP_LOGI(TAG, "Place finger on sensor and hold steady...");
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Collect initial buffer of samples
    ESP_LOGI(TAG, "Collecting %d samples...", BUFFER_LENGTH);
    
    for (int i = 0; i < BUFFER_LENGTH; i++) {
        esp_err_t ret = max30102_read_register(dev_handle, REG_FIFO_DATA, data, 6);
        if (ret == ESP_OK) {
            red_buffer[i] = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
            ir_buffer[i]  = ((uint32_t)data[3] << 16) | ((uint32_t)data[4] << 8) | data[5];
            red_buffer[i] &= 0x3FFFF;
            ir_buffer[i] &= 0x3FFFF;
            
            if (i % 25 == 0) {
                ESP_LOGI(TAG, "Sample %d: RED=%lu, IR=%lu", i, red_buffer[i], ir_buffer[i]);
            }
        } else {
            ESP_LOGE(TAG, "Read failed at sample %d", i);
            i--; // Retry
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz
    }

    // Run Maxim's algorithm on the collected data
    ESP_LOGI(TAG, "Running algorithm...");
    maxim_heart_rate_and_oxygen_saturation(
        ir_buffer, 
        BUFFER_LENGTH,
        red_buffer,
        &spo2, 
        &spo2_valid,
        &heart_rate, 
        &hr_valid
    );

    // Display results
    if (hr_valid) {
        ESP_LOGI(TAG, "Heart Rate: %ld BPM (valid)", heart_rate);
    } else {
        ESP_LOGW(TAG, "Heart Rate: INVALID (poor signal quality)");
    }
    
    if (spo2_valid) {
        ESP_LOGI(TAG, "SpO2: %ld%% (valid)", spo2);
    } else {
        ESP_LOGW(TAG, "SpO2: INVALID (poor signal quality)");
    }
    if (ir_buffer[BUFFER_LENGTH - 1] < 5000) {
    ESP_LOGW(TAG, "No finger detected");
}

    // Continuous reading mode - rolling buffer
    int sample_idx = 0;
    int update_count = 0;
    
    while (1) {
        esp_err_t ret = max30102_read_register(dev_handle, REG_FIFO_DATA, data, 6);
        if (ret == ESP_OK) {
            // Shift buffer and add new sample
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
            
            // Run algorithm every 25 samples (4 times per second)
            if (sample_idx >= 25) {
                maxim_heart_rate_and_oxygen_saturation(
                    ir_buffer, 
                    BUFFER_LENGTH,
                    red_buffer,
                    &spo2, 
                    &spo2_valid,
                    &heart_rate, 
                    &hr_valid
                );
                
                ESP_LOGI(TAG, "HR: %s%ld BPM | SpO2: %s%ld%% | IR: %lu",
                        hr_valid ? "" : "INVALID ",
                        heart_rate,
                        spo2_valid ? "" : "INVALID ",
                        spo2,
                        ir_buffer[BUFFER_LENGTH - 1]);
                
                sample_idx = 0;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}








//this one works fine------------------------------------------------------------------------------------------------------------------------>
// #include "algorithm.h"
// #include <stdint.h>

// void maxim_heart_rate_and_oxygen_saturation(uint32_t *pun_ir_buffer, int32_t n_ir_buffer_length, 
//                                             uint32_t *pun_red_buffer, int32_t *pn_spo2, 
//                                             int8_t *pch_spo2_valid, int32_t *pn_heart_rate, 
//                                             int8_t *pch_hr_valid)
// {
//   uint32_t un_ir_mean;
//   int32_t k, n_i_ratio_count;
//   int32_t i, n_exact_ir_valley_locs_count, n_middle_idx;
//   int32_t n_th1, n_npks;   
//   int32_t an_ir_valley_locs[15];
//   int32_t n_peak_interval_sum;
  
//   int32_t n_y_ac, n_x_ac;
//   int32_t n_y_dc_max, n_x_dc_max; 
//   int32_t n_y_dc_max_idx, n_x_dc_max_idx; 
//   int32_t an_ratio[5], n_ratio_average; 
//   int32_t n_nume, n_denom;
//   int32_t an_x[BUFFER_SIZE]; // ir
//   int32_t an_y[BUFFER_SIZE]; // red

//   // Calculate DC mean and subtract DC from ir
//   un_ir_mean = 0; 
//   for (k = 0; k < n_ir_buffer_length; k++) 
//     un_ir_mean += pun_ir_buffer[k];
//   un_ir_mean = un_ir_mean / n_ir_buffer_length;
  
//   // Remove DC and invert signal so we can use peak detector as valley detector
//   for (k = 0; k < n_ir_buffer_length; k++)  
//     an_x[k] = un_ir_mean - pun_ir_buffer[k];

//   // 4 pt Moving Average
//   for (k = 0; k < BUFFER_SIZE_MA4; k++) {
//     an_x[k] = (an_x[k] + an_x[k+1] + an_x[k+2] + an_x[k+3]) / 4;        
//   }
  
//   // Calculate threshold  
//   n_th1 = 0; 
//   for (k = 0; k < BUFFER_SIZE_MA4; k++) {
//     n_th1 += an_x[k];
//   }
//   n_th1 = n_th1 / BUFFER_SIZE_MA4;
//   if (n_th1 < 30) n_th1 = 30; // min allowed
//   if (n_th1 > 60) n_th1 = 60; // max allowed

//   for (k = 0; k < 15; k++) 
//     an_ir_valley_locs[k] = 0;
  
//   // Since we flipped signal, we use peak detector as valley detector
//   maxim_find_peaks(an_ir_valley_locs, &n_npks, an_x, BUFFER_SIZE_MA4, n_th1, 4, 15);
  
//   n_peak_interval_sum = 0;
//   if (n_npks >= 2) {
//     for (k = 1; k < n_npks; k++) 
//       n_peak_interval_sum += (an_ir_valley_locs[k] - an_ir_valley_locs[k-1]);
//     n_peak_interval_sum = n_peak_interval_sum / (n_npks - 1);
//     *pn_heart_rate = (int32_t)((FS * 60) / n_peak_interval_sum);
//     *pch_hr_valid = 1;
//   } else { 
//     *pn_heart_rate = -999; // Unable to calculate
//     *pch_hr_valid = 0;
//   }

//   // Load raw value again for SPO2 calculation: RED(=y) and IR(=X)
//   for (k = 0; k < n_ir_buffer_length; k++) {
//     an_x[k] = pun_ir_buffer[k]; 
//     an_y[k] = pun_red_buffer[k]; 
//   }

//   // Find precise min near an_ir_valley_locs
//   n_exact_ir_valley_locs_count = n_npks; 
  
//   n_ratio_average = 0; 
//   n_i_ratio_count = 0; 
//   for (k = 0; k < 5; k++) 
//     an_ratio[k] = 0;
    
//   for (k = 0; k < n_exact_ir_valley_locs_count; k++) {
//     if (an_ir_valley_locs[k] > BUFFER_SIZE) {
//       *pn_spo2 = -999; // Valley loc is out of range
//       *pch_spo2_valid = 0; 
//       return;
//     }
//   }
  
//   // Find max between two valley locations 
//   for (k = 0; k < n_exact_ir_valley_locs_count - 1; k++) {
//     n_y_dc_max = -16777216; 
//     n_x_dc_max = -16777216; 
    
//     if (an_ir_valley_locs[k+1] - an_ir_valley_locs[k] > 3) {
//       for (i = an_ir_valley_locs[k]; i < an_ir_valley_locs[k+1]; i++) {
//         if (an_x[i] > n_x_dc_max) {
//           n_x_dc_max = an_x[i]; 
//           n_x_dc_max_idx = i;
//         }
//         if (an_y[i] > n_y_dc_max) {
//           n_y_dc_max = an_y[i]; 
//           n_y_dc_max_idx = i;
//         }
//       }
      
//       n_y_ac = (an_y[an_ir_valley_locs[k+1]] - an_y[an_ir_valley_locs[k]]) * 
//                (n_y_dc_max_idx - an_ir_valley_locs[k]);
//       n_y_ac = an_y[an_ir_valley_locs[k]] + n_y_ac / (an_ir_valley_locs[k+1] - an_ir_valley_locs[k]); 
//       n_y_ac = an_y[n_y_dc_max_idx] - n_y_ac;
      
//       n_x_ac = (an_x[an_ir_valley_locs[k+1]] - an_x[an_ir_valley_locs[k]]) * 
//                (n_x_dc_max_idx - an_ir_valley_locs[k]);
//       n_x_ac = an_x[an_ir_valley_locs[k]] + n_x_ac / (an_ir_valley_locs[k+1] - an_ir_valley_locs[k]); 
//       n_x_ac = an_x[n_y_dc_max_idx] - n_x_ac;
      
//       n_nume = (n_y_ac * n_x_dc_max) >> 7;
//       n_denom = (n_x_ac * n_y_dc_max) >> 7;
      
//       if (n_denom > 0 && n_i_ratio_count < 5 && n_nume != 0) {   
//         an_ratio[n_i_ratio_count] = (n_nume * 100) / n_denom;
//         n_i_ratio_count++;
//       }
//     }
//   }
  
//   // Choose median value
//   maxim_sort_ascend(an_ratio, n_i_ratio_count);
//   n_middle_idx = n_i_ratio_count / 2;

//   if (n_middle_idx > 1)
//     n_ratio_average = (an_ratio[n_middle_idx-1] + an_ratio[n_middle_idx]) / 2;
//   else
//     n_ratio_average = an_ratio[n_middle_idx];

//   if (n_ratio_average > 2 && n_ratio_average < 184) {
//     *pn_spo2 = uch_spo2_table[n_ratio_average];
//     *pch_spo2_valid = 1;
//   } else {
//     *pn_spo2 = -999;
//     *pch_spo2_valid = 0; 
//   }
// }

// void maxim_find_peaks(int32_t *pn_locs, int32_t *n_npks, int32_t *pn_x, 
//                       int32_t n_size, int32_t n_min_height, int32_t n_min_distance, 
//                       int32_t n_max_num)
// {
//   maxim_peaks_above_min_height(pn_locs, n_npks, pn_x, n_size, n_min_height);
//   maxim_remove_close_peaks(pn_locs, n_npks, pn_x, n_min_distance);
//   *n_npks = (*n_npks > n_max_num) ? n_max_num : *n_npks;
// }

// void maxim_peaks_above_min_height(int32_t *pn_locs, int32_t *n_npks, int32_t *pn_x, 
//                                    int32_t n_size, int32_t n_min_height)
// {
//   int32_t i = 1, n_width;
//   *n_npks = 0;
  
//   while (i < n_size - 1) {
//     if (pn_x[i] > n_min_height && pn_x[i] > pn_x[i-1]) {
//       n_width = 1;
//       while (i + n_width < n_size && pn_x[i] == pn_x[i+n_width])
//         n_width++;
//       if (pn_x[i] > pn_x[i+n_width] && (*n_npks) < 15) {
//         pn_locs[(*n_npks)++] = i;
//         i += n_width + 1;
//       } else
//         i += n_width;
//     } else
//       i++;
//   }
// }

// void maxim_remove_close_peaks(int32_t *pn_locs, int32_t *pn_npks, int32_t *pn_x, 
//                                int32_t n_min_distance)
// {
//   int32_t i, j, n_old_npks, n_dist;
    
//   maxim_sort_indices_descend(pn_x, pn_locs, *pn_npks);

//   for (i = -1; i < *pn_npks; i++) {
//     n_old_npks = *pn_npks;
//     *pn_npks = i + 1;
//     for (j = i + 1; j < n_old_npks; j++) {
//       n_dist = pn_locs[j] - (i == -1 ? -1 : pn_locs[i]);
//       if (n_dist > n_min_distance || n_dist < -n_min_distance)
//         pn_locs[(*pn_npks)++] = pn_locs[j];
//     }
//   }

//   maxim_sort_ascend(pn_locs, *pn_npks);
// }

// void maxim_sort_ascend(int32_t *pn_x, int32_t n_size) 
// {
//   int32_t i, j, n_temp;
//   for (i = 1; i < n_size; i++) {
//     n_temp = pn_x[i];
//     for (j = i; j > 0 && n_temp < pn_x[j-1]; j--)
//       pn_x[j] = pn_x[j-1];
//     pn_x[j] = n_temp;
//   }
// }

// void maxim_sort_indices_descend(int32_t *pn_x, int32_t *pn_indx, int32_t n_size)
// {
//   int32_t i, j, n_temp;
//   for (i = 1; i < n_size; i++) {
//     n_temp = pn_indx[i];
//     for (j = i; j > 0 && pn_x[n_temp] > pn_x[pn_indx[j-1]]; j--)
//       pn_indx[j] = pn_indx[j-1];
//     pn_indx[j] = n_temp;
//   }
// }