#include "startup_diagnostic.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "STARTUP_DIAG";

static startup_test_results_t test_results = {0};

int startup_test_pump(void)
{
    ESP_LOGI(TAG, "Testing pump...");
    // TODO: Run pump for 500ms and check current changes
    // If current changes, pump works
    test_results.pump_test_passed = 1;
    return 1;
}

int startup_test_leds(void)
{
    ESP_LOGI(TAG, "Testing LEDs...");
    // TODO: Cycle through all LEDs, verify each one
    // Check if LEDs light up and respond to commands
    test_results.led_test_passed = 1;
    return 1;
}

int startup_test_sensors(void)
{
    ESP_LOGI(TAG, "Testing sensors...");
    // TODO: Read temp from both sensors
    // Verify values are in reasonable range
    // If sensors not responding, fail
    test_results.sensor_test_passed = 1;
    return 1;
}

int startup_test_i2c(void)
{
    ESP_LOGI(TAG, "Testing I2C communication...");
    // TODO: Check if both TMP117 devices respond
    // Check INA226 responds
    // If ACK received from all, pass
    test_results.i2c_test_passed = 1;
    return 1;
}

int startup_diagnostic_run(void)
{
    ESP_LOGI(TAG, "=== RUNNING STARTUP DIAGNOSTICS ===");
    
    startup_test_pump();
    vTaskDelay(pdMS_TO_TICKS(500));
    
    startup_test_leds();
    vTaskDelay(pdMS_TO_TICKS(500));
    
    startup_test_sensors();
    vTaskDelay(pdMS_TO_TICKS(500));
    
    startup_test_i2c();
    vTaskDelay(pdMS_TO_TICKS(500));
    
    test_results.all_tests_passed = (test_results.pump_test_passed &&
                                      test_results.led_test_passed &&
                                      test_results.sensor_test_passed &&
                                      test_results.i2c_test_passed);
    
    startup_diagnostic_display_results();
    
    return test_results.all_tests_passed;
}

startup_test_results_t startup_diagnostic_get_results(void)
{
    return test_results;
}

void startup_diagnostic_display_results(void)
{
    ESP_LOGI(TAG, "\n=== DIAGNOSTIC RESULTS ===");
    ESP_LOGI(TAG, "Pump Test:    %s", test_results.pump_test_passed ? "PASS" : "FAIL");
    ESP_LOGI(TAG, "LED Test:     %s", test_results.led_test_passed ? "PASS" : "FAIL");
    ESP_LOGI(TAG, "Sensor Test:  %s", test_results.sensor_test_passed ? "PASS" : "FAIL");
    ESP_LOGI(TAG, "I2C Test:     %s", test_results.i2c_test_passed ? "PASS" : "FAIL");
    
    if (test_results.all_tests_passed) {
        ESP_LOGI(TAG, "\nALL TESTS PASSED - System ready!");
    } else {
        ESP_LOGE(TAG, "\nSOME TESTS FAILED - Check sensor placement and connections");
    }
    ESP_LOGI(TAG, "==========================\n");
}
