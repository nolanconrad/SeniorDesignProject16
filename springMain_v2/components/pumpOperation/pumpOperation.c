#include "pumpOperation.h"

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"

// Arduino Nano ESP32: A0 maps to GPIO1.
#define PUMP_PWM_GPIO GPIO_NUM_1
#define PWM_TIMER LEDC_TIMER_0
#define PWM_MODE LEDC_LOW_SPEED_MODE
#define PWM_CHANNEL LEDC_CHANNEL_0
// Use high-enough PWM frequency for motor drive while preserving duty resolution.
#define PWM_RESOLUTION LEDC_TIMER_10_BIT
#define PWM_FREQ_HZ 20000

static const char *TAG = "pump_operation";
static bool s_initialized = false;
static uint8_t s_duty_percent = 0;
static const uint32_t PWM_MAX_DUTY = (1U << PWM_RESOLUTION) - 1U;

esp_err_t pump_operation_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    ledc_timer_config_t ledc_timer = {
        .speed_mode = PWM_MODE,
        .timer_num = PWM_TIMER,
        .duty_resolution = PWM_RESOLUTION,
        .freq_hz = PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };

    esp_err_t err = ledc_timer_config(&ledc_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config failed: %s", esp_err_to_name(err));
        return err;
    }

    ledc_channel_config_t ledc_channel = {
        .speed_mode = PWM_MODE,
        .channel = PWM_CHANNEL,
        .timer_sel = PWM_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = PUMP_PWM_GPIO,
        .duty = 0,
        .hpoint = 0,
        .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
    };

    err = ledc_channel_config(&ledc_channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_channel_config failed: %s", esp_err_to_name(err));
        return err;
    }

    s_initialized = true;
    s_duty_percent = 0;

    ESP_LOGI(TAG, "Pump PWM ready on A0(GPIO%d) at %d Hz", PUMP_PWM_GPIO, PWM_FREQ_HZ);
    return ESP_OK;
}

esp_err_t pump_operation_set_duty_percent(uint8_t duty_percent)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (duty_percent > 100) {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t duty = (duty_percent * PWM_MAX_DUTY + 50U) / 100U;

    esp_err_t err = ledc_set_duty(PWM_MODE, PWM_CHANNEL, duty);
    if (err != ESP_OK) {
        return err;
    }

    err = ledc_update_duty(PWM_MODE, PWM_CHANNEL);
    if (err == ESP_OK) {
        s_duty_percent = duty_percent;
    }

    return err;
}

esp_err_t pump_operation_stop(void)
{
    return pump_operation_set_duty_percent(0);
}

uint8_t pump_operation_get_duty_percent(void)
{
    return s_duty_percent;
}