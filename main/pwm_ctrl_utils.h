#ifndef LPWM_CTRL_UTILS_H
#define LPWM_CTRL_UTILS_H

#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "hw_config.h"
#include "func_config.h"

typedef enum {
    AE_CLICK,
    AE_OK,
    AE_INVALID,
    AE_ERROR,
    AE_ACTION,
    AE_COUNT,
} actuator_event_t;

inline QueueHandle_t actuator_queue = xQueueCreate(5, sizeof(actuator_event_t));

void push_actuator_event(actuator_event_t event);
void pwm_ctrl_init(void);
void led_set_color(uint8_t red, uint8_t green, uint8_t blue);
void led_off(void);

#ifdef DEVICE_ARM
void motor_set(uint8_t strength);
void motor_strong(void);
void motor_stop(void);
void motor_gentle(void);
#endif

#endif // LPWM_CTRL_UTILS_H