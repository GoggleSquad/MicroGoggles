// #include "max30102_api.h"
// #include "i2c_api.h"


// void max30102_init(max_config *configuration)
// {
// 	write_max30102_reg(configuration->data2,  REG_INTR_ENABLE_2);
// 	write_max30102_reg(configuration->data3,  REG_FIFO_WR_PTR);
// 	write_max30102_reg(configuration->data4,  REG_OVF_COUNTER);
// 	write_max30102_reg(configuration->data5,  REG_FIFO_RD_PTR);
// 	write_max30102_reg(configuration->data6,  REG_FIFO_CONFIG);
// 	write_max30102_reg(configuration->data7,  REG_MODE_CONFIG);
// 	write_max30102_reg(configuration->data8,  REG_SPO2_CONFIG);
// 	write_max30102_reg(configuration->data9,  REG_LED1_PA);
// 	write_max30102_reg(configuration->data10, REG_LED2_PA);
// 	write_max30102_reg(configuration->data11, REG_PILOT_PA);
// 	write_max30102_reg(configuration->data12, REG_MULTI_LED_CTRL1);
// 	write_max30102_reg(configuration->data13, REG_MULTI_LED_CTRL2);
// }


// void read_max30102_fifo(int32_t *red_data, int32_t *ir_data)
// {
// 	uint8_t un_temp[6];
// 	uint8_t fifo_reg = REG_FIFO_DATA;

//     i2c_sensor_write(&fifo_reg, 1);

//     i2c_sensor_read(&un_temp, 6);
//      *red_data += un_temp[0] << 16;
//      *red_data += un_temp[1] << 8;
//      *red_data += un_temp[2];

//      *ir_data += un_temp[3] << 16;
//      *ir_data += un_temp[4] << 8;
//      *ir_data += un_temp[5];
// }


// void read_max30102_reg(uint8_t reg_addr, uint8_t *data_reg, size_t bytes_to_read)
// {
// 	i2c_sensor_write(&reg_addr, 1);
// 	i2c_sensor_read(data_reg, bytes_to_read);
// }


// void write_max30102_reg(uint8_t command, uint8_t reg)
// {
// 	uint8_t data[2];
// 	data[0] = reg;
// 	data[1] = command;
// 	i2c_sensor_write(data, 2);
// }


// float get_max30102_temp()
// {
// 	uint8_t int_temp;
// 	uint8_t decimal_temp;
// 	float temp = 0;
// 	write_max30102_reg(1, REG_TEMP_CONFIG);
// 	read_max30102_reg(REG_TEMP_INTR, &int_temp, 1);
// 	read_max30102_reg(REG_TEMP_FRAC, &decimal_temp, 1);
// 	temp = (int_temp + ((float)decimal_temp/10));
// 	return temp;
// }


#include "max30102_api.h"
#include "i2c_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "MAX30102";

void max30102_init(max_config *configuration)
{
    esp_err_t ret;
    
    // Reset the device first
    uint8_t reset_val = 0x40;
    ret = write_max30102_reg(reset_val, REG_MODE_CONFIG);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset MAX30102");
        return;
    }
    
    vTaskDelay(pdMS_TO_TICKS(100));  // Wait for reset
    
    // Configure registers in order
    ret = write_max30102_reg(configuration->data6, REG_FIFO_CONFIG);
    if (ret == ESP_OK) ESP_LOGI(TAG, "FIFO Config: OK");
    
    ret = write_max30102_reg(configuration->data7, REG_MODE_CONFIG);
    if (ret == ESP_OK) ESP_LOGI(TAG, "Mode Config: OK");
    
    ret = write_max30102_reg(configuration->data8, REG_SPO2_CONFIG);
    if (ret == ESP_OK) ESP_LOGI(TAG, "SPO2 Config: OK");
    
    ret = write_max30102_reg(configuration->data9, REG_LED1_PA);
    if (ret == ESP_OK) ESP_LOGI(TAG, "LED1 PA: OK");
    
    ret = write_max30102_reg(configuration->data10, REG_LED2_PA);
    if (ret == ESP_OK) ESP_LOGI(TAG, "LED2 PA: OK");
    
    // Clear FIFO pointers
    write_max30102_reg(0x00, REG_FIFO_WR_PTR);
    write_max30102_reg(0x00, REG_OVF_COUNTER);
    write_max30102_reg(0x00, REG_FIFO_RD_PTR);
    
    // Enable interrupts (optional)
    write_max30102_reg(configuration->data1, REG_INTR_ENABLE_1);
    write_max30102_reg(configuration->data2, REG_INTR_ENABLE_2);
    
    ESP_LOGI(TAG, "MAX30102 initialization complete");
}

void read_max30102_fifo(int32_t *red_data, int32_t *ir_data)
{
    uint8_t un_temp[6];
    uint8_t fifo_reg = REG_FIFO_DATA;
    
    esp_err_t ret = i2c_sensor_write(&fifo_reg, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write FIFO register address");
        return;
    }
    
    ret = i2c_sensor_read(un_temp, 6);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read FIFO data");
        return;
    }
    
    // RED LED data (first 3 bytes)
    *red_data = ((uint32_t)un_temp[0] << 16) | ((uint32_t)un_temp[1] << 8) | un_temp[2];
    *red_data &= 0x3FFFF;  // 18-bit data
    
    // IR LED data (last 3 bytes)
    *ir_data = ((uint32_t)un_temp[3] << 16) | ((uint32_t)un_temp[4] << 8) | un_temp[5];
    *ir_data &= 0x3FFFF;  // 18-bit data
}

void read_max30102_reg(uint8_t reg_addr, uint8_t *data_reg, size_t bytes_to_read)
{
    i2c_sensor_write(&reg_addr, 1);
    i2c_sensor_read(data_reg, bytes_to_read);
}

esp_err_t write_max30102_reg(uint8_t command, uint8_t reg)
{
    uint8_t data[2];
    data[0] = reg;
    data[1] = command;
    return i2c_sensor_write(data, 2);
}

float get_max30102_temp()
{
    uint8_t int_temp;
    uint8_t decimal_temp;
    float temp = 0;
    
    // Trigger temperature measurement
    write_max30102_reg(0x01, REG_TEMP_CONFIG);
    
    // Wait for conversion
    vTaskDelay(pdMS_TO_TICKS(100));
    
    read_max30102_reg(REG_TEMP_INTR, &int_temp, 1);
    read_max30102_reg(REG_TEMP_FRAC, &decimal_temp, 1);
    
    temp = (float)int_temp + ((float)decimal_temp * 0.0625f);
    
    return temp;
}

// Test function to verify sensor communication
bool max30102_test_connection()
{
    uint8_t part_id = 0;
    uint8_t rev_id = 0;
    
    read_max30102_reg(REG_PART_ID, &part_id, 1);
    read_max30102_reg(REG_REV_ID, &rev_id, 1);
    
    ESP_LOGI(TAG, "Part ID: 0x%02X (expected 0x15)", part_id);
    ESP_LOGI(TAG, "Rev ID: 0x%02X", rev_id);
    
    return (part_id == 0x15);  // MAX30102 Part ID
}