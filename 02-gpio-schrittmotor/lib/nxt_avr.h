#ifndef __NXT_AVR_H__
#define __NXT_AVR_H__

#include <stdint.h>
#include "../main.h"

button_t  nxt_avr_get_buttons(void);
uint16_t  nxt_avr_get_battery_mv(void);
uint16_t  nxt_avr_get_battery_raw(void);
battery_t nxt_avr_get_battery_type(void);
uint8_t   nxt_avr_get_avr_version(void);
uint16_t  nxt_avr_get_sensor_adc_raw(sensor_t sens);
void      nxt_avr_set_sensor_power  (sensor_t sens, sensor_power_t sensor_power);
void      nxt_avr_power_down(void);
void      nxt_avr_firmware_update_enter(void);
void      nxt_avr_set_motor(motor_t motor, int power_percent, motor_zustand_t brake);
void      nxt_avr_init(uint8_t pwm_frequency_kHz);

#endif
