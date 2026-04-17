#include "system_monitor.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "SYSTEM_MONITOR";

#define TEMP_CRITICAL_HIGH 55.0f
#define TEMP_CRITICAL_LOW -10.0f
#define TEMP_COOLDOWN_TARGET 45.0f
#define CURRENT_CRITICAL 25.0f
#define COOLDOWN_CHECK_INTERVAL 5000  // ms

static system_state_t current_state = SYSTEM_OK;
static error_code_t last_error = ERROR_NONE;
static int cooldown_active = 0;
static float cooldown_target = TEMP_COOLDOWN_TARGET;
static TickType_t cooldown_start_time = 0;

void system_monitor_init(void)
{
    ESP_LOGI(TAG, "System monitor initialized");
    current_state = SYSTEM_OK;
    last_error = ERROR_NONE;
}

system_state_t system_monitor_get_state(void)
{
    return current_state;
}

void system_monitor_update(float temp1, float temp2, float current, float power)
{
    // Temperature critical check
    if (temp1 > TEMP_CRITICAL_HIGH || temp2 > TEMP_CRITICAL_HIGH) {
        ESP_LOGE(TAG, "CRITICAL TEMPERATURE DETECTED: T1=%.2f T2=%.2f", temp1, temp2);
        current_state = SYSTEM_COOLDOWN;
        last_error = ERROR_TEMP_CRITICAL;
        system_enter_cooldown(temp1 > temp2 ? temp1 : temp2, TEMP_COOLDOWN_TARGET);
        return;
    }

    // Current/pump failure check
    if (current > CURRENT_CRITICAL) {
        ESP_LOGE(TAG, "PUMP FAILURE - Excessive current: %.2f A", current);
        current_state = SYSTEM_PUMP_FAILURE;
        last_error = ERROR_PUMP_BLOCKED;
        return;
    }

    // Check if we can recover from cooldown
    if (cooldown_active) {
        float avg_temp = (temp1 + temp2) / 2.0f;
        if (avg_temp < cooldown_target) {
            ESP_LOGI(TAG, "Temperature acceptable, exiting cooldown");
            cooldown_active = 0;
            current_state = SYSTEM_OK;
        }
    }
}

void system_enter_cooldown(float current_temp, float target_temp)
{
    ESP_LOGW(TAG, "Entering COOLDOWN phase - Current: %.2f°C Target: %.2f°C", current_temp, target_temp);
    cooldown_active = 1;
    cooldown_target = target_temp;
    cooldown_start_time = xTaskGetTickCount();
    current_state = SYSTEM_COOLDOWN;
}

int system_is_in_cooldown(void)
{
    return cooldown_active;
}

int system_is_safe(void)
{
    return (current_state == SYSTEM_OK && !cooldown_active);
}

void system_log_error(error_code_t code, const char *message)
{
    ESP_LOGE(TAG, "[ERROR %d] %s", code, message);
    last_error = code;
}

error_code_t system_get_last_error(void)
{
    return last_error;
}

void system_emergency_shutdown(const char *reason)
{
    ESP_LOGE(TAG, "EMERGENCY SHUTDOWN: %s", reason);
    current_state = SYSTEM_SHUTDOWN;
    // TODO: Turn off pump, flash alert LED, disable all operations
}

void system_reset(void)
{
    ESP_LOGI(TAG, "System reset");
    current_state = SYSTEM_OK;
    last_error = ERROR_NONE;
    cooldown_active = 0;
}
