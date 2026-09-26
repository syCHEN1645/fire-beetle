#ifndef PWM_CTRL_UTILS_H
#define PWM_CTRL_UTILS_H

#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "hw_config.h"
#include "func_config.h"
#include "data_struct_utils.h"

void pwm_ctrl_init(void);
void led_set_color(uint8_t red, uint8_t green, uint8_t blue);
void led_off(void);

#ifdef DEVICE_ARM
void motor_set(uint8_t strength);
void motor_strong(void);
void motor_stop(void);
void motor_gentle(void);
#endif

#endif // PWM_CTRL_UTILS_H