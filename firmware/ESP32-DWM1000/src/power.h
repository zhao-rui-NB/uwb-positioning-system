#ifndef __POWER_H__
#define __POWER_H__

#include <Arduino.h>


#ifdef __cplusplus
extern "C"{
#endif

#define PIN_BATTERY_ADC 34



void update_battery_voltage_mv(uint16_t voltage_mv);
uint16_t get_battery_voltage_mv() ;
void power_init();
void power_task();








#ifdef __cplusplus
}
#endif



#endif

