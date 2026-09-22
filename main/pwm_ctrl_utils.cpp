#include "pwm_ctrl_utils.h"

void push_actuator_event(actuator_event_t event) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(actuator_queue, &event, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void pwm_ctrl_init() {
    // Configure PWM timer
    ledc_timer_config_t timer_config = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_8_BIT,
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = 5000,
        .clk_cfg          = LEDC_AUTO_CLK,
        .deconfigure      = false,
    };

    ESP_ERROR_CHECK(
        ledc_timer_config(&timer_config)
    );

    // Red
    ledc_channel_config_t red_config = {
        .gpio_num   = LED_R_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,
        .hpoint     = 0,
        .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_ALLOW_PD,
        .flags      = 0
    };

    ESP_ERROR_CHECK(
        ledc_channel_config(&red_config)
    );

    // Green
    ledc_channel_config_t green_config = {
        .gpio_num   = LED_G_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_1,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,
        .hpoint     = 0,
        .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_ALLOW_PD,
        .flags      = 0
    };

    ESP_ERROR_CHECK(
        ledc_channel_config(&green_config)
    );

    // Blue
    ledc_channel_config_t blue_config = {
        .gpio_num   = LED_B_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_2,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,
        .hpoint     = 0,
        .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_ALLOW_PD,
        .flags      = 0
    };

    ESP_ERROR_CHECK(
        ledc_channel_config(&blue_config)
    );

#ifdef DEVICE_ARM
    // Motor
    ledc_channel_config_t motor_config = {
        .gpio_num   = MOTOR_CTRL_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_3,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,
        .hpoint     = 0,
        .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_ALLOW_PD,
        .flags      = 0
    };

    ESP_ERROR_CHECK(
        ledc_channel_config(&motor_config)
    );
#endif
}

/// @param r Red component (0-255)
/// @param g Green component (0-255)
/// @param b Blue component (0-255)
void led_set_color(uint8_t r, uint8_t g, uint8_t b) {
    // set, then update
    ESP_ERROR_CHECK(
        ledc_set_duty(
            LEDC_LOW_SPEED_MODE,
            LEDC_CHANNEL_0,
            r
        )
    );
    ESP_ERROR_CHECK(
        ledc_update_duty(
            LEDC_LOW_SPEED_MODE,
            LEDC_CHANNEL_0
        )
    );

    ESP_ERROR_CHECK(
        ledc_set_duty(
            LEDC_LOW_SPEED_MODE,
            LEDC_CHANNEL_1,
            g
        )
    );
    ESP_ERROR_CHECK(
        ledc_update_duty(
            LEDC_LOW_SPEED_MODE,
            LEDC_CHANNEL_1
        )
    );

    ESP_ERROR_CHECK(
        ledc_set_duty(
            LEDC_LOW_SPEED_MODE,
            LEDC_CHANNEL_2,
            b
        )
    );
    ESP_ERROR_CHECK(
        ledc_update_duty(
            LEDC_LOW_SPEED_MODE,
            LEDC_CHANNEL_2
        )
    );
}

void led_off(void) {
    led_set_color(0, 0, 0);
}

#ifdef DEVICE_ARM
void motor_set(uint8_t strength) {
    ESP_ERROR_CHECK(
        ledc_set_duty(
            LEDC_LOW_SPEED_MODE,
            LEDC_CHANNEL_3,
            strength
        )
    );

    ESP_ERROR_CHECK(
        ledc_update_duty(
            LEDC_LOW_SPEED_MODE,
            LEDC_CHANNEL_3
        )
    );
}

void motor_strong(void) {
    motor_set(MOTOR_STRONG);
}

void motor_stop(void) {
    motor_set(MOTOR_STOP);
}

void motor_gentle(void) {
    motor_set(MOTOR_GENTLE);
}
#endif
