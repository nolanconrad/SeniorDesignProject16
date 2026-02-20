#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_intr_types.h"

// Component headers
#include "main.h"
#include "system_startUp.h"
#include "pump_Operation.h"
#include "i2c_with_tmp117.h"
#include "ina226.h"
#include "system_monitor.h"
#include "startup_diagnostic.h"
#include "mode_manager.h"

static const char *TAG = "MAIN";

#define TEMP_CRITICAL_HIGH 55.0f
#define TEMP_COOLDOWN_TARGET 45.0f

// System state variables
static operating_mode_t current_operating_mode = MODE_STARTUP;
static int system_initialized = 0;
static int startup_tests_passed = 0;

// Error callbacks
void on_sensor_error(int device)
{
    ESP_LOGE(TAG, "SENSOR ERROR on device %d", device);
    mode_manager_set_mode(MODE_ERROR);
}

void on_sensor_overheat(int device, float temperature)
{
    ESP_LOGE(TAG, "OVERHEAT: Device %d at %.2f°C", device, temperature);
    pump_Set_Mode(0);
    mode_manager_set_mode(MODE_COOLDOWN);
}

void on_high_current_alert(float current)
{
    ESP_LOGE(TAG, "PUMP FAILURE - Current: %.2f A", current);
    pump_Set_Mode(0);
    mode_manager_set_mode(MODE_ERROR);
}

// Initialize all components
void initialize_system(void)
{
    ESP_LOGI(TAG, "=== SYSTEM STARTUP ===");
    
    mode_manager_init();
    system_monitor_init();
    tmp117_init();
    ina226_init();
    pump_Operation_init();
    
    // Register callbacks
    tmp117_register_error_callback(on_sensor_error);
    tmp117_register_overheat_callback(on_sensor_overheat);
    ina226_register_high_current_callback(on_high_current_alert);
    
    ina226_set_current_threshold(20.0f);
    ina226_set_power_threshold(150.0f);
    
    system_initialized = 1;
}

// Run startup tests
void run_startup_tests(void)
{
    ESP_LOGI(TAG, "=== STARTUP TESTS ===");
    
    mode_manager_set_mode(MODE_STARTUP);
    system_startUp();
    startup_tests_passed = startup_diagnostic_run();
    
    if (!startup_tests_passed) {
        ESP_LOGE(TAG, "Tests failed - Manual mode only");
    }
    
    // Default to manual mode - user must enable automatic
    mode_manager_set_mode(MODE_MANUAL_LOW);
}

// Automatic mode task
void automatic_mode_task(void *pvParameters)
{
    while (mode_manager_is_automatic_enabled()) {
        float temp1 = tmp117_read_temperature();
        float temp2 = tmp117_read_temperature_device2();
        float current = ina226_read_current();
        float power = ina226_read_power();
        
        system_monitor_update(temp1, temp2, current, power);
        
        if (system_monitor_get_state() == SYSTEM_COOLDOWN) {
            pump_Set_Mode(0);
        } else if (system_monitor_get_state() == SYSTEM_PUMP_FAILURE) {
            pump_Set_Mode(0);
            break;
        } else {
            // TODO: Read moisture sensor and control pump
        }
        
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// Main loop
void app_main(void)
{
    initialize_system();
    run_startup_tests();
    
    ESP_LOGI(TAG, "=== MAIN LOOP ===\n");
    
    while (1) {
        operating_mode_t mode = mode_manager_get_mode();
        
        switch(mode) {
            case MODE_MANUAL_LOW:
            case MODE_MANUAL_MEDIUM:
            case MODE_MANUAL_HIGH:
                // Manual mode - user controls pump speed
                pump_Set_Mode(mode_manager_get_manual_level());
                // TODO: Handle mode cycle button interrupt
                vTaskDelay(pdMS_TO_TICKS(1000));
                break;
            
            case MODE_AUTOMATIC:
                // Automatic mode - system controls based on sensors
                static xTaskHandle auto_task = NULL;
                if (auto_task == NULL) {
                    xTaskCreate(automatic_mode_task, "auto_mode", 2048, NULL, 5, &auto_task);
                }
                vTaskDelay(pdMS_TO_TICKS(1000));
                break;
            
            case MODE_COOLDOWN:
                // Temperature too high - wait for cooldown
                pump_Set_Mode(0);
                float temp = (tmp117_read_temperature() + tmp117_read_temperature_device2()) / 2.0f;
                if (temp < TEMP_COOLDOWN_TARGET) {
                    ESP_LOGI(TAG, "Temperature OK - resuming");
                    system_monitor_init();
                    mode_manager_set_mode(MODE_MANUAL_LOW);
                }
                vTaskDelay(pdMS_TO_TICKS(2000));
                break;
            
            case MODE_ERROR:
                // Error state - require reset
                pump_Set_Mode(0);
                // TODO: Check for reset button
                vTaskDelay(pdMS_TO_TICKS(1000));
                break;
            
            default:
                vTaskDelay(pdMS_TO_TICKS(100));
                break;
        }
    }
}

