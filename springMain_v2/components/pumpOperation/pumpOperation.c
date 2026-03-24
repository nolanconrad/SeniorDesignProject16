#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"

#define PUMP_PIN         GPIO_NUM_18
#define PUMP_PWM_MODE    LEDC_LOW_SPEED_MODE
#define PUMP_PWM_TIMER   LEDC_TIMER_0
#define PUMP_PWM_CHANNEL LEDC_CHANNEL_0
#define PUMP_PWM_FREQ    5000
#define PUMP_PWM_RES     LEDC_TIMER_8_BIT   // duty: 0 to 255

static int pump_mode = 0;

// Example placeholders for automatic mode
static int temp1 = 0;
static int threshold1 = 50;

static void pump_set_duty(uint8_t duty)
{
    ledc_set_duty(PUMP_PWM_MODE, PUMP_PWM_CHANNEL, duty);
    ledc_update_duty(PUMP_PWM_MODE, PUMP_PWM_CHANNEL);
}

void pump_Operation_init(void)
{
    ledc_timer_config_t timer_conf = {
        .speed_mode = PUMP_PWM_MODE,
        .timer_num = PUMP_PWM_TIMER,
        .duty_resolution = PUMP_PWM_RES,
        .freq_hz = PUMP_PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);

    ledc_channel_config_t channel_conf = {
        .gpio_num = PUMP_PIN,
        .speed_mode = PUMP_PWM_MODE,
        .channel = PUMP_PWM_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = PUMP_PWM_TIMER,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&channel_conf);

    pump_mode = 0;
    pump_set_duty(0);
}

void pump_Set_Mode(int mode)
{
    pump_mode = mode;
}

void man_Pump_On(int intensity)
{
    if (intensity < 0) intensity = 0;
    if (intensity > 255) intensity = 255;

    pump_set_duty((uint8_t)intensity);
}

void automatic_Pump_On(void)
{
    if (temp1 > threshold1 + 10) {
        pump_set_duty(255);   // high
    } else if (temp1 > threshold1) {
        pump_set_duty(160);   // medium
    } else {
        pump_set_duty(80);    // low
    }
}

void pump_off(void)
{
    pump_set_duty(0);
}

void pump_Operation(void)
{
    switch (pump_mode) {
        case 1:
            automatic_Pump_On();
            break;

        case 2:
            man_Pump_On(85);    // low
            break;

        case 3:
            man_Pump_On(170);   // medium
            break;

        case 4:
            man_Pump_On(255);   // high
            break;

        case 5:
        default:
            pump_off();
            break;
    }
}