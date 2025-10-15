#ifndef MAIN_H
#define MAIN_H

#include "esp_err.h"
#include "max30102_api.h"

void sensor_data_reader(void *pvParameters);
void fill_buffers_data();

#define BUFFER_SIZE 128

// Optimized MAX30102 configuration for heart rate and SpO2
max_config max30102_configuration = {

    .INT_EN_1.A_FULL_EN         = 0,
    .INT_EN_1.PPG_RDY_EN        = 0,
    .INT_EN_1.ALC_OVF_EN        = 0,
    .INT_EN_1.PROX_INT_EN       = 0,

    .INT_EN_2.DIE_TEMP_RDY_EN   = 0,

    .FIFO_WRITE_PTR.FIFO_WR_PTR = 0,
    .OVEF_COUNTER.OVF_COUNTER   = 0,
    .FIFO_READ_PTR.FIFO_RD_PTR  = 0,

    .FIFO_CONF.SMP_AVE          = 0b000,  // No averaging
    .FIFO_CONF.FIFO_ROLLOVER_EN = 1,
    .FIFO_CONF.FIFO_A_FULL      = 0,

    .MODE_CONF.SHDN             = 0,
    .MODE_CONF.RESET            = 0,
    .MODE_CONF.MODE             = 0b011,  // SpO2 mode

    .SPO2_CONF.SPO2_ADC_RGE     = 0b10,   // 8192 nA range
    .SPO2_CONF.SPO2_SR          = 0b010,  // 100 samples/sec
    .SPO2_CONF.LED_PW           = 0b11,   // 411 μs pulse width

    .LED1_PULSE_AMP.LED1_PA     = 0x3F,   // RED LED: ~12.5 mA
    .LED2_PULSE_AMP.LED2_PA     = 0x3F,   // IR LED: ~12.5 mA

    .PROX_LED_PULS_AMP.PILOT_PA = 0X00,

    .MULTI_LED_CONTROL1.SLOT2   = 0,
    .MULTI_LED_CONTROL1.SLOT1   = 0,
    .MULTI_LED_CONTROL2.SLOT4   = 0,
    .MULTI_LED_CONTROL2.SLOT3   = 0,
};

#endif