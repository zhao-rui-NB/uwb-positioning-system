#include <Arduino.h>
#include "power.h"

#include "driver/adc.h"
#include "esp_adc_cal.h"

uint16_t battery_voltage_mv = 0;
static esp_adc_cal_characteristics_t adc_chars;
void update_battery_voltage_mv(uint16_t voltage_mv) {
    battery_voltage_mv = voltage_mv;
}

uint16_t get_battery_voltage_mv() {
    return battery_voltage_mv;
}

void power_init() {
    pinMode(PIN_BATTERY_ADC, INPUT);

    // 設定 ADC 解析度與衰減值
    analogReadResolution(12);
    analogSetAttenuation(ADC_6db);  // 可量測到約 3.3V

    // 初始化校正
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_6, ADC_WIDTH_BIT_12, 1100, &adc_chars);


    // start task

    xTaskCreatePinnedToCore(
        (TaskFunction_t)power_task,   /* Task function. */
        "power_task",                 /* name of task. */
        2048,                         /* Stack size of task */
        NULL,                         /* parameter of the task */
        1,                            /* priority of the task */
        NULL,                         /* Task handle to keep track of created task */
        1);                           /* pin task to core 1 */

}

void power_task() {
    while (1) {
        int adc_value = analogRead(PIN_BATTERY_ADC);
        uint32_t adc_mv = esp_adc_cal_raw_to_voltage(adc_value, &adc_chars);

        // assuming a 3.3V reference and a voltage divider that halves the battery voltage
        uint16_t mv = adc_mv * 3 - 100; // 簡單校正(亂減)
        update_battery_voltage_mv(mv);
        vTaskDelay(pdMS_TO_TICKS(1000));
        // Serial.printf("[Power] Battery Voltage: %d mV\n", get_battery_voltage_mv());
    }
}

