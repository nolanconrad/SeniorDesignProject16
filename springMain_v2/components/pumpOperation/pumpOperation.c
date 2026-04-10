#include "pumpOperation.h"

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"

// Arduino Nano ESP32: A0 maps to GPIO1.
#define PUMP_PWM_GPIO GPIO_NUM_1

static const char *TAG = "pump_operation";
static bool s_initialized = false;
static uint8_t s_duty_percent = 0;

static esp_err_t pump_apply_output_percent(uint8_t duty_percent)
{
    if (duty_percent > 100) {
        return ESP_ERR_INVALID_ARG;
    }

    // This module intentionally supports only OFF or FULL-ON output.
    uint8_t applied_percent = (duty_percent == 0) ? 0 : 100;
    esp_err_t err = gpio_set_level(PUMP_PWM_GPIO, applied_percent == 100 ? 1 : 0);
    if (err == ESP_OK) {
        s_duty_percent = applied_percent;
    }
    return err;
}

esp_err_t pump_operation_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    gpio_config_t pump_gpio_cfg = {
        .pin_bit_mask = (1ULL << PUMP_PWM_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t err = gpio_config(&pump_gpio_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "pump gpio config failed: %s", esp_err_to_name(err));
        return err;
    }

    err = gpio_set_level(PUMP_PWM_GPIO, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "failed to drive pump pin low: %s", esp_err_to_name(err));
        return err;
    }

    s_initialized = true;
    s_duty_percent = 0;
    ESP_LOGI(TAG, "Pump output ready on A0(GPIO%d) in ON/OFF mode", PUMP_PWM_GPIO);

    return ESP_OK;
}

esp_err_t pump_operation_set_duty_percent(uint8_t duty_percent)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    return pump_apply_output_percent(duty_percent);
}

esp_err_t pump_operation_stop(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    return pump_apply_output_percent(0);
}

uint8_t pump_operation_get_duty_percent(void)
{
    return s_duty_percent;
}