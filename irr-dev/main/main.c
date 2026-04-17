#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_intr_types.h"
#include "esp_mac.h"

// Component headers
#include "main.h"
#include "system_startUp.h"
#include "pump_Operation.h"
#include "i2c_with_tmp117.h"
#include "ina226.h"
#include "system_monitor.h"
#include "startup_diagnostic.h"
#include "mode_manager.h"
#include "ble_handler.h"

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
    ble_init();

    // Force pump off before recalibrating INA226 zero-current offset.
    pump_Set_Mode(0);
    vTaskDelay(pdMS_TO_TICKS(150));
    ina226_recalibrate_zero_offset();
    
    // Register callbacks
    tmp117_register_error_callback(on_sensor_error);
    tmp117_register_overheat_callback(on_sensor_overheat);
    ina226_register_high_current_callback(on_high_current_alert);
    
    ina226_set_current_threshold(3.6f);
    ina226_set_power_threshold(25.0f);
    
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
        float current_pump = ina226_read_current_device(INA226_DEVICE_PRIMARY);
        float power_pump = ina226_read_power_device(INA226_DEVICE_PRIMARY);
        float current_logic = ina226_read_current_device(INA226_DEVICE_LOGIC);
        float power_logic = ina226_read_power_device(INA226_DEVICE_LOGIC);
        int pwm_mode = mode_manager_get_manual_level();
        
        ESP_LOGI(TAG, "PWM: %d | TMP1: %.2f°C | TMP2: %.2f°C | PUMP: I=%.3f A P=%.3f W | LOGIC: I=%.3f A P=%.3f W", 
                 pwm_mode, temp1, temp2, current_pump, power_pump, current_logic, power_logic);
        
        system_monitor_update(temp1, temp2, current_pump, power_pump);
        
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

    esp_log_level_set("*", ESP_LOG_DEBUG);
    
    // Get and log chip ID
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_EFUSE_FACTORY);
    uint64_t chipid = ((uint64_t)mac[0] << 40) | ((uint64_t)mac[1] << 32) | 
                      ((uint64_t)mac[2] << 24) | ((uint64_t)mac[3] << 16) | 
                      ((uint64_t)mac[4] << 8) | mac[5];
    ESP_LOGI(TAG, "ESP32 Chip ID = %04X%08X", (uint16_t)(chipid >> 32), (uint32_t)chipid);

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
                float temp1 = tmp117_read_temperature();
                float temp2 = tmp117_read_temperature_device2();
                float current_pump = ina226_read_current_device(INA226_DEVICE_PRIMARY);
                float power_pump = ina226_read_power_device(INA226_DEVICE_PRIMARY);
                float current_logic = ina226_read_current_device(INA226_DEVICE_LOGIC);
                float power_logic = ina226_read_power_device(INA226_DEVICE_LOGIC);
                int pwm_mode = mode_manager_get_manual_level();
                if (current_pump > 0 || power_pump > 0) {
                    ESP_LOGI(TAG, "PWM: %d | TMP1: %.2f°C | TMP2: %.2f°C | PUMP: I=%.3f A P=%.3f W | LOGIC: I=%.3f A P=%.3f W", 
                             pwm_mode, temp1, temp2, current_pump, power_pump, current_logic, power_logic);
                } else {
                    ESP_LOGW(TAG, "PWM: %d | TMP1: %.2f°C | TMP2: %.2f°C | PUMP: I=%.3f A (CHECK) P=%.3f W | LOGIC: I=%.3f A P=%.3f W", 
                             pwm_mode, temp1, temp2, current_pump, power_pump, current_logic, power_logic);
                }
                // TODO: Handle mode cycle button interrupt
                vTaskDelay(pdMS_TO_TICKS(1000));
                break;
            
            case MODE_AUTOMATIC:
                // Automatic mode - system controls based on sensors
                static TaskHandle_t auto_task = NULL;
                if (auto_task == NULL) {
                    xTaskCreate(automatic_mode_task, "auto_mode", 2048, NULL, 5, &auto_task);
                }
                vTaskDelay(pdMS_TO_TICKS(1000));
                break;
            
            case MODE_COOLDOWN:
                // Temperature too high - wait for cooldown
                pump_Set_Mode(0);
                float temp_cd1 = tmp117_read_temperature();
                float temp_cd2 = tmp117_read_temperature_device2();
                float current_cd_pump = ina226_read_current_device(INA226_DEVICE_PRIMARY);
                float power_cd_pump = ina226_read_power_device(INA226_DEVICE_PRIMARY);
                float current_cd_logic = ina226_read_current_device(INA226_DEVICE_LOGIC);
                float power_cd_logic = ina226_read_power_device(INA226_DEVICE_LOGIC);
                ESP_LOGI(TAG, "COOLDOWN | PWM: 0 | TMP1: %.2f°C | TMP2: %.2f°C | PUMP: I=%.3f A P=%.3f W | LOGIC: I=%.3f A P=%.3f W", 
                         temp_cd1, temp_cd2, current_cd_pump, power_cd_pump, current_cd_logic, power_cd_logic);
                if (temp_cd1 < TEMP_COOLDOWN_TARGET && temp_cd2 < TEMP_COOLDOWN_TARGET) {
                    ESP_LOGI(TAG, "Temperature OK - resuming");
                    system_monitor_init();
                    mode_manager_set_mode(MODE_MANUAL_LOW);
                }
                vTaskDelay(pdMS_TO_TICKS(2000));
                break;
            
            case MODE_ERROR:
                // Error state - require reset
                pump_Set_Mode(0);
                float temp_err1 = tmp117_read_temperature();
                float temp_err2 = tmp117_read_temperature_device2();
                float current_err_pump = ina226_read_current_device(INA226_DEVICE_PRIMARY);
                float power_err_pump = ina226_read_power_device(INA226_DEVICE_PRIMARY);
                float current_err_logic = ina226_read_current_device(INA226_DEVICE_LOGIC);
                float power_err_logic = ina226_read_power_device(INA226_DEVICE_LOGIC);
                ESP_LOGI(TAG, "ERROR | PWM: 0 | TMP1: %.2f°C | TMP2: %.2f°C | PUMP: I=%.3f A P=%.3f W | LOGIC: I=%.3f A P=%.3f W", 
                         temp_err1, temp_err2, current_err_pump, power_err_pump, current_err_logic, power_err_logic);
                // TODO: Check for reset button
                vTaskDelay(pdMS_TO_TICKS(1000));
                break;
            
            default:
                vTaskDelay(pdMS_TO_TICKS(100));
                break;
        }
    }
}

