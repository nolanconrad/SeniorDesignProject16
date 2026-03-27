#include "pumpOperation.h"

#include <stdbool.h>
#include <stdint.h>

#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"

// Arduino Nano ESP32: A0 maps to GPIO1.
#define PUMP_PWM_GPIO GPIO_NUM_1
#define PUMP_PWM_MODE LEDC_LOW_SPEED_MODE
#define PUMP_PWM_TIMER LEDC_TIMER_1
#define PUMP_PWM_CHANNEL LEDC_CHANNEL_1
#define PUMP_PWM_FREQ_HZ 20000
#define PUMP_PWM_RESOLUTION LEDC_TIMER_10_BIT
#define PUMP_PWM_MAX_DUTY ((1U << PUMP_PWM_RESOLUTION) - 1U)

static const char *TAG = "pump_operation";
static bool s_initialized = false;
static uint8_t s_duty_percent = 0;

static esp_err_t pump_apply_duty_percent(uint8_t duty_percent)
{
    if (duty_percent > 100) {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t duty = (PUMP_PWM_MAX_DUTY * duty_percent) / 100U;
    esp_err_t err = ledc_set_duty(PUMP_PWM_MODE, PUMP_PWM_CHANNEL, duty);
    if (err != ESP_OK) {
        return err;
    }

    err = ledc_update_duty(PUMP_PWM_MODE, PUMP_PWM_CHANNEL);
    if (err == ESP_OK) {
        s_duty_percent = duty_percent;
    }

    return err;
}

esp_err_t pump_operation_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    ledc_timer_config_t timer_conf = {
        .speed_mode = PUMP_PWM_MODE,
        .duty_resolution = PUMP_PWM_RESOLUTION,
        .timer_num = PUMP_PWM_TIMER,
        .freq_hz = PUMP_PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false,
    };

    esp_err_t err = ledc_timer_config(&timer_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config failed: %s", esp_err_to_name(err));
        return err;
    }

    ledc_channel_config_t channel_conf = {
        .gpio_num = PUMP_PWM_GPIO,
        .speed_mode = PUMP_PWM_MODE,
        .channel = PUMP_PWM_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = PUMP_PWM_TIMER,
        .duty = 0,
        .hpoint = 0,
        .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
        .flags = {
            .output_invert = 0,
        },
    };

    err = ledc_channel_config(&channel_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_channel_config failed: %s", esp_err_to_name(err));
        return err;
    }

    s_initialized = true;
    s_duty_percent = 0;
    ESP_LOGI(TAG, "DRV8871 PWM ready on A0(GPIO%d), %d Hz", PUMP_PWM_GPIO, PUMP_PWM_FREQ_HZ);

    return ESP_OK;
}

esp_err_t pump_operation_set_duty_percent(uint8_t duty_percent)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    return pump_apply_duty_percent(duty_percent);
}

esp_err_t pump_operation_stop(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    return pump_apply_duty_percent(0);
}

uint8_t pump_operation_get_duty_percent(void)
{
    return s_duty_percent;
}