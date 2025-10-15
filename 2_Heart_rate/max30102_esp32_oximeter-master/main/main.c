#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "nvs_flash.h"
#include "main.h"
#include "max30102_api.h"
#include "algorithm.h"
#include "i2c_api.h"
#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>

static const char *TAG = "MAIN";

TaskHandle_t sensor_reader_handle = NULL;

int32_t red_data = 0;
int32_t ir_data = 0;
int32_t red_data_buffer[BUFFER_SIZE];
int32_t ir_data_buffer[BUFFER_SIZE];
double auto_correlationated_data[BUFFER_SIZE];

#define DELAY_AMOSTRAGEM 40
#define DEBUG false  // Set to true for detailed output

void app_main(void)
{
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "  MAX30102 Heart Rate & SpO2 Monitor");
    ESP_LOGI(TAG, "===========================================");
    
    // Initialize NVS (required for some ESP32 functions)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize I2C
    ret = i2c_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C initialization failed: %s", esp_err_to_name(ret));
        ESP_LOGE(TAG, "Check your wiring:");
        ESP_LOGE(TAG, "  SDA -> GPIO 21");
        ESP_LOGE(TAG, "  SCL -> GPIO 22");
        ESP_LOGE(TAG, "  VIN -> 3.3V");
        ESP_LOGE(TAG, "  GND -> GND");
        return;
    }
    ESP_LOGI(TAG, "✓ I2C initialized successfully");
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Test MAX30102 connection
    if (max30102_test_connection()) {
        ESP_LOGI(TAG, "✓ MAX30102 sensor detected!");
    } else {
        ESP_LOGE(TAG, "✗ MAX30102 sensor not found!");
        ESP_LOGE(TAG, "Troubleshooting:");
        ESP_LOGE(TAG, "  1. Check wiring connections");
        ESP_LOGE(TAG, "  2. Verify sensor address (0x57)");
        ESP_LOGE(TAG, "  3. Check power supply (3.3V)");
        ESP_LOGE(TAG, "  4. Try different I2C pins");
        return;
    }
    
    // Initialize MAX30102
    max30102_init(&max30102_configuration);
    init_time_array();
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "System ready! Place your finger on the sensor...");
    ESP_LOGI(TAG, "");
    
    // Create sensor reading task
    xTaskCreatePinnedToCore(
        sensor_data_reader, 
        "SensorReader", 
        8192,
        NULL, 
        5,
        &sensor_reader_handle, 
        1
    );
}

void sensor_data_reader(void *pvParameters)
{
    uint64_t ir_mean;
    uint64_t red_mean;
    float temperature;
    double r0_autocorrelation;
    int measurement_count = 0;
    int no_finger_count = 0;
    
    for(;;){
        measurement_count++;
        
        // Fill buffers with data
        fill_buffers_data();
        
        // Check if we have valid data (finger detected)
        bool valid_data = false;
        int valid_samples = 0;
        for(int i = 0; i < BUFFER_SIZE; i++){
            if(ir_data_buffer[i] > 50000){  // Threshold for finger detection
                valid_samples++;
            }
        }
        
        // Need at least 50% valid samples
        if(valid_samples > BUFFER_SIZE / 2){
            valid_data = true;
        }
        
        if(!valid_data){
            no_finger_count++;
            if(no_finger_count % 10 == 0){  // Print every 10 attempts
                ESP_LOGW(TAG, "No finger detected. Please place finger firmly on sensor.");
                ESP_LOGW(TAG, "IR sample: %" PRId32 " (need > 50000)", ir_data_buffer[0]);
            }
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        
        // Reset counter when finger is detected
        no_finger_count = 0;
        
        // Get temperature
        temperature = get_max30102_temp();
        
        // Process signals
        remove_dc_part(ir_data_buffer, red_data_buffer, &ir_mean, &red_mean);
        remove_trend_line(ir_data_buffer);
        remove_trend_line(red_data_buffer);
        
        double pearson_correlation = correlation_datay_datax(red_data_buffer, ir_data_buffer);
        int heart_rate = calculate_heart_rate(ir_data_buffer, &r0_autocorrelation, auto_correlationated_data);
        
        // Calculate SpO2 if signals are well correlated
        if(pearson_correlation >= 0.7 && heart_rate > 40 && heart_rate < 200){
            double spo2 = spo2_measurement(ir_data_buffer, red_data_buffer, ir_mean, red_mean);
            
            // Clamp SpO2 to reasonable range
            if(spo2 > 100.0) spo2 = 100.0;
            if(spo2 < 70.0) spo2 = 70.0;
            
            // Display results
            printf("\n");
            printf("Measurement #%d              \n", measurement_count);
            printf("Heart Rate:  %3d bpm             \n", heart_rate);
            printf("SpO2:        %5.1f%%              \n", spo2);
            printf("Temperature: %5.1f°C             \n", temperature);
            printf("Correlation:  %.3f               \n", pearson_correlation);
            printf("\n");
            
        } else {
            ESP_LOGW(TAG, "Poor signal quality - HR: %d, Correlation: %.3f", heart_rate, pearson_correlation);
            if(pearson_correlation < 0.7){
                ESP_LOGW(TAG, "  → Signals not well correlated. Hold finger steady.");
            }
            if(heart_rate <= 40 || heart_rate >= 200){
                ESP_LOGW(TAG, "  → Heart rate out of range. Adjust finger pressure.");
            }
        }
        
#if DEBUG
        // Print sample data for debugging
        ESP_LOGI(TAG, "Raw sample data (first 10):");
        for(int i = 0; i < 10 && i < BUFFER_SIZE; i++){
            printf("  [%d] IR:%" PRId32 " RED:%" PRId32 " AC:%.3f\n", 
                i, ir_data_buffer[i], red_data_buffer[i], auto_correlationated_data[i]);
        }
        ESP_LOGI(TAG, "IR Mean: %" PRIu64 ", RED Mean: %" PRIu64, ir_mean, red_mean);
#endif
        
        vTaskDelay(pdMS_TO_TICKS(500));  // Delay between measurements
    }
}

void fill_buffers_data()
{
    for(int i = 0; i < BUFFER_SIZE; i++){
        read_max30102_fifo(&red_data, &ir_data);
        ir_data_buffer[i] = ir_data;
        red_data_buffer[i] = red_data;
        
        ir_data = 0;
        red_data = 0;
        
        vTaskDelay(pdMS_TO_TICKS(DELAY_AMOSTRAGEM));
    }
}