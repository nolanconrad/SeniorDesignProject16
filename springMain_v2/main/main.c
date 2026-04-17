#include <stdio.h>
#include <stdbool.h>

#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "ina226.h"
#include "tmp117.h"
#include "i2c_bus.h"
#include "pumpOperation.h"
#include "ble_handler.h"

// --- Smoothing filter initialization flags ---
static bool filter_ina1_initialized = false;
static bool filter_ina2_initialized = false;

// --- Smoothing filter globals ---
static float filtered_ina1_current_ma = 0.0f;
static float filtered_ina2_current_ma = 0.0f;
static float filtered_ina1_power_mw = 0.0f;
static float filtered_ina2_power_mw = 0.0f;

// --- Synchronized INA reading globals ---
static SemaphoreHandle_t ina_read_semaphore = NULL;
static ina226_measurement_t synchronized_ina1_measurement = {0};
static ina226_measurement_t synchronized_ina2_measurement = {0};
static esp_err_t synchronized_ina1_error = ESP_OK;
static esp_err_t synchronized_ina2_error = ESP_OK;

static const char *TAG = "SYSTEM";

// Button and LED Configuration
#define BUTTON_D2 GPIO_NUM_5        // Mode toggle: Manual/Automatic (pin D2)
#define BUTTON_A2 GPIO_NUM_3        // Pump increase button (pin A2) - manual mode only
#define BUTTON_A3 GPIO_NUM_4        // Pump decrease button (pin A3) - manual mode only

// Manual mode indicator LEDs (D9, D8, D7) - show pump level
#define LED_INDICATOR_D2 GPIO_NUM_18  // Pump level indicator 1 (Arduino D9)
#define LED_INDICATOR_D3 GPIO_NUM_17  // Pump level indicator 2 (Arduino D8)
#define LED_INDICATOR_D4 GPIO_NUM_10  // Pump level indicator 3 (Arduino D7)

// Auto mode status LEDs (D3, D4) - stay on during auto
#define LED_AUTO_STATUS_1 GPIO_NUM_6   // Status LED 1 (Arduino D3) - stays on
#define LED_AUTO_STATUS_2 GPIO_NUM_7   // Status LED 2 (Arduino D4) - stays on

// Auto mode animation LEDs (D5-D9) - progressive animation
#define LED_ANIM_D5 GPIO_NUM_8   // Animation LED 1 (Arduino D5)
#define LED_ANIM_D6 GPIO_NUM_9   // Animation LED 2 (Arduino D6)
#define LED_ANIM_D7 GPIO_NUM_10  // Animation LED 3 (Arduino D7)
#define LED_ANIM_D8 GPIO_NUM_17  // Animation LED 4 (Arduino D8)
#define LED_ANIM_D9 GPIO_NUM_18  // Animation LED 5 (Arduino D9)

#define PUMP_INCREMENT 10           // 10% per button press
#define PUMP_START_PERCENT 50       // Starting pump intensity

// I2C Sensor Addresses
#define INA226_1_ADDR 0x40
#define INA226_2_ADDR 0x41
#define TMP117_ADDR TMP117_I2C_ADDR_DEFAULT

// INA226 hardware configuration
#define INA226_PUMP_SHUNT_RESISTANCE_OHM 0.1f
#define INA226_LOGIC_SHUNT_RESISTANCE_OHM 0.1f

// Timing configuration
#define BUTTON_POLL_INTERVAL_MS 50
#define SENSOR_READ_INTERVAL_MS 2000

// PWM-synchronized INA reading configuration
#define PUMP_PWM_FREQ_HZ 500
#define PWM_PERIOD_MS (1000 / PUMP_PWM_FREQ_HZ)
#define INA_READ_OFFSET_MS 3
#define INA_READING_TASK_PRIORITY (tskIDLE_PRIORITY + 2)
#define INA_READING_TASK_STACK_SIZE 4096

typedef enum {
    CONTROL_MANUAL = 0,
    CONTROL_AUTOMATIC = 1
} control_mode_t;

// Temperature thresholds for automatic control (Fahrenheit)
#define TEMP_STEP_98F 98.0f
#define TEMP_STEP_99F 99.0f
#define TEMP_STEP_100F 100.0f
#define TEMP_STEP_101F 101.0f
#define TEMP_STEP_102F 102.0f

#define PUMP_10_PERCENT 10
#define PUMP_25_PERCENT 25
#define PUMP_50_PERCENT 50
#define PUMP_75_PERCENT 75
#define PUMP_100_PERCENT 100

// LED animation for auto mode
#define LED_ANIM_INTERVAL_MS 1000
#define LED_FLASH_INTERVAL_MS 250

typedef struct {
    gpio_num_t pin;
    bool stable_state;      // true=released, false=pressed
    bool current_state;
} button_state_t;

// Forward declarations
static void update_indicator_leds(uint32_t pump_percent);
static void update_flash_leds(uint32_t current_ms);
static void update_status_leds(uint32_t current_ms);
static void update_animation_leds(uint32_t current_ms);
static void refresh_pump_output_state(void);

// PWM-synchronized INA reading callback
static void pwm_sync_timer_callback(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(ina_read_semaphore, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

// Dedicated task for synchronized INA readings
static void ina_reading_task(void *arg)
{
    while (1) {
        if (xSemaphoreTake(ina_read_semaphore, portMAX_DELAY) == pdTRUE) {
            vTaskDelay(pdMS_TO_TICKS(INA_READ_OFFSET_MS));

            synchronized_ina1_error = ina226_read_measurement(
                INA226_1_ADDR,
                INA226_PUMP_SHUNT_RESISTANCE_OHM,
                &synchronized_ina1_measurement);

            synchronized_ina2_error = ina226_read_measurement(
                INA226_2_ADDR,
                INA226_LOGIC_SHUNT_RESISTANCE_OHM,
                &synchronized_ina2_measurement);

            ESP_LOGD(TAG, "INA readings synchronized at PWM cycle + 3ms");
        }
    }
}

// Global state
static uint32_t current_pump_percent = PUMP_START_PERCENT;
static control_mode_t control_mode = CONTROL_MANUAL;
static uint32_t last_flash_toggle_ms = 0;
static bool flash_state = false;
static uint32_t last_status_led_toggle_ms = 0;
static bool status_led_state = false;
static uint32_t last_anim_update_ms = 0;
static uint8_t anim_led_index = 0;
static button_state_t button_d2 = {BUTTON_D2, true, true};
static button_state_t button_a2 = {BUTTON_A2, true, true};
static button_state_t button_a3 = {BUTTON_A3, true, true};
static esp_timer_handle_t pwm_sync_timer = NULL;

static void toggle_led(void)
{
    control_mode = (control_mode == CONTROL_MANUAL) ? CONTROL_AUTOMATIC : CONTROL_MANUAL;

    refresh_pump_output_state();

    if (control_mode == CONTROL_MANUAL) {
        update_indicator_leds(current_pump_percent);
        ESP_LOGI(TAG, "Control mode: MANUAL - Use A2/A3 buttons to adjust pump");
    } else {
        ESP_LOGI(TAG, "Control mode: AUTOMATIC - Pump controlled by temperature");
    }
}

static void update_pump_auto(float temp_f)
{
    uint32_t new_pump_percent = current_pump_percent;

    if (temp_f <= TEMP_STEP_98F) {
        new_pump_percent = PUMP_10_PERCENT;
    }
    else if (temp_f < TEMP_STEP_99F) {
        new_pump_percent = PUMP_10_PERCENT;
    }
    else if (temp_f < TEMP_STEP_100F) {
        new_pump_percent = PUMP_25_PERCENT;
    }
    else if (temp_f < TEMP_STEP_101F) {
        new_pump_percent = PUMP_50_PERCENT;
    }
    else if (temp_f <= TEMP_STEP_102F) {
        new_pump_percent = PUMP_75_PERCENT;
    }
    else {
        new_pump_percent = PUMP_100_PERCENT;
    }

    if (new_pump_percent != current_pump_percent) {
        current_pump_percent = new_pump_percent;
        refresh_pump_output_state();
        ESP_LOGI(TAG, "Auto mode: Temp %.2f°F -> Pump %lu%%",
                 temp_f, (unsigned long)new_pump_percent);
    }
}

static void update_animation_leds(uint32_t current_ms)
{
    gpio_set_level(LED_AUTO_STATUS_1, 1);
    gpio_set_level(LED_AUTO_STATUS_2, 1);

    if (current_ms - last_anim_update_ms >= LED_ANIM_INTERVAL_MS) {
        last_anim_update_ms = current_ms;
        anim_led_index = (anim_led_index + 1) % 5;

        gpio_set_level(LED_ANIM_D5, 1);
        gpio_set_level(LED_ANIM_D6, anim_led_index >= 1 ? 1 : 0);
        gpio_set_level(LED_ANIM_D7, anim_led_index >= 2 ? 1 : 0);
        gpio_set_level(LED_ANIM_D8, anim_led_index >= 3 ? 1 : 0);
        gpio_set_level(LED_ANIM_D9, anim_led_index >= 4 ? 1 : 0);

        ESP_LOGD(TAG, "Auto mode animation frame: %d", anim_led_index);
    }
}

static void update_indicator_leds(uint32_t pump_percent)
{
    if (pump_percent == 0) {
        gpio_set_level(LED_INDICATOR_D2, 0);
        gpio_set_level(LED_INDICATOR_D3, 0);
        gpio_set_level(LED_INDICATOR_D4, 0);
        ESP_LOGI(TAG, "Indicator LEDs: OFF");
    }
    else if (pump_percent <= 30) {
        gpio_set_level(LED_INDICATOR_D2, 1);
        gpio_set_level(LED_INDICATOR_D3, 0);
        gpio_set_level(LED_INDICATOR_D4, 0);
        ESP_LOGI(TAG, "Indicator LEDs: D2 ON");
    }
    else if (pump_percent <= 60) {
        gpio_set_level(LED_INDICATOR_D2, 1);
        gpio_set_level(LED_INDICATOR_D3, 1);
        gpio_set_level(LED_INDICATOR_D4, 0);
        ESP_LOGI(TAG, "Indicator LEDs: D2 D3 ON");
    }
    else if (pump_percent < 100) {
        gpio_set_level(LED_INDICATOR_D2, 1);
        gpio_set_level(LED_INDICATOR_D3, 1);
        gpio_set_level(LED_INDICATOR_D4, 1);
        ESP_LOGI(TAG, "Indicator LEDs: D2 D3 D4 ON");
    }
    else {
        ESP_LOGI(TAG, "Indicator LEDs: FLASHING MODE");
    }
}

static void update_flash_leds(uint32_t current_ms)
{
    if (current_pump_percent == 100) {
        if (current_ms - last_flash_toggle_ms >= LED_FLASH_INTERVAL_MS) {
            flash_state = !flash_state;
            last_flash_toggle_ms = current_ms;

            gpio_set_level(LED_INDICATOR_D2, flash_state ? 1 : 0);
            gpio_set_level(LED_INDICATOR_D3, flash_state ? 1 : 0);
            gpio_set_level(LED_INDICATOR_D4, flash_state ? 1 : 0);
        }
    }
}

static void update_status_leds(uint32_t current_ms)
{
    if (control_mode == CONTROL_MANUAL) {
        if (current_ms - last_status_led_toggle_ms >= LED_FLASH_INTERVAL_MS) {
            status_led_state = !status_led_state;
            last_status_led_toggle_ms = current_ms;

            gpio_set_level(LED_AUTO_STATUS_1, status_led_state ? 1 : 0);
            gpio_set_level(LED_AUTO_STATUS_2, status_led_state ? 1 : 0);
        }
    }
}

static void init_ina_sync_reading(void)
{
    ina_read_semaphore = xSemaphoreCreateBinary();
    if (ina_read_semaphore == NULL) {
        ESP_LOGE(TAG, "Failed to create INA read semaphore");
        return;
    }
    ESP_LOGD(TAG, "INA read semaphore created");

    const esp_timer_create_args_t timer_args = {
        .callback = pwm_sync_timer_callback,
        .name = "pwm_sync_timer",
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
    };

    esp_err_t err = esp_timer_create(&timer_args, &pwm_sync_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create PWM sync timer: %s", esp_err_to_name(err));
        return;
    }

    uint64_t timer_period_us = (PWM_PERIOD_MS * 1000);
    err = esp_timer_start_periodic(pwm_sync_timer, timer_period_us);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start PWM sync timer: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "PWM sync timer started: %llu us period (%.1f Hz)",
             timer_period_us, 1000000.0f / timer_period_us);

    TaskHandle_t task_handle;
    BaseType_t task_result = xTaskCreate(
        ina_reading_task,
        "ina_reading_task",
        INA_READING_TASK_STACK_SIZE,
        NULL,
        INA_READING_TASK_PRIORITY,
        &task_handle);

    if (task_result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create INA reading task");
        return;
    }

    ESP_LOGI(TAG, "INA synchronized reading system initialized");
}

static void init_buttons_and_led(void)
{
    gpio_config_t indicator_config = {
        .pin_bit_mask = (1ULL << LED_INDICATOR_D2) | (1ULL << LED_INDICATOR_D3) | (1ULL << LED_INDICATOR_D4),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&indicator_config);
    gpio_set_level(LED_INDICATOR_D2, 0);
    gpio_set_level(LED_INDICATOR_D3, 0);
    gpio_set_level(LED_INDICATOR_D4, 0);

    gpio_config_t status_led_config = {
        .pin_bit_mask = (1ULL << LED_AUTO_STATUS_1) | (1ULL << LED_AUTO_STATUS_2),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&status_led_config);
    gpio_set_level(LED_AUTO_STATUS_1, 0);
    gpio_set_level(LED_AUTO_STATUS_2, 0);

    gpio_config_t anim_led_config = {
        .pin_bit_mask = (1ULL << LED_ANIM_D5) | (1ULL << LED_ANIM_D6) | (1ULL << LED_ANIM_D7) | (1ULL << LED_ANIM_D8) | (1ULL << LED_ANIM_D9),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&anim_led_config);
    gpio_set_level(LED_ANIM_D5, 0);
    gpio_set_level(LED_ANIM_D6, 0);
    gpio_set_level(LED_ANIM_D7, 0);
    gpio_set_level(LED_ANIM_D8, 0);
    gpio_set_level(LED_ANIM_D9, 0);

    gpio_config_t button_config = {
        .pin_bit_mask = (1ULL << BUTTON_D2) | (1ULL << BUTTON_A2) | (1ULL << BUTTON_A3),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&button_config);

    ESP_LOGI(TAG, "All LEDs and buttons initialized");
}

static bool button_pressed(button_state_t *btn)
{
    bool new_state = gpio_get_level(btn->pin) == 0;

    if (new_state == false && btn->stable_state == true) {
        btn->stable_state = false;
        return true;
    }

    if (new_state == true && btn->stable_state == false) {
        btn->stable_state = true;
    }

    return false;
}

static void refresh_pump_output_state(void)
{
    if (pump_operation_set_duty_percent((uint8_t)current_pump_percent) != ESP_OK) {
        ESP_LOGW(TAG, "Failed to apply pump output state");
        return;
    }

    if (control_mode == CONTROL_MANUAL) {
        update_indicator_leds(current_pump_percent);
    }
}

void app_main(void)
{
    esp_err_t err = i2c_bus_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C bus: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "I2C bus initialized");

    ESP_LOGI(TAG, "Scanning I2C bus...");
    i2c_bus_scan(pdMS_TO_TICKS(50));

    err = pump_operation_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize pump output: %s", esp_err_to_name(err));
        return;
    }

    init_ina_sync_reading();
    init_buttons_and_led();

    esp_err_t terr = tmp117_init(TMP117_ADDR);
    if (terr == ESP_OK) {
        ESP_LOGI(TAG, "TMP117 found and initialized at 0x%02X", TMP117_ADDR);
    } else {
        ESP_LOGE(TAG, "TMP117 not found at 0x%02X: %s", TMP117_ADDR, esp_err_to_name(terr));
    }

    err = ina226_init(INA226_1_ADDR, INA226_PUMP_SHUNT_RESISTANCE_OHM);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "PUMP (INA1, 0x%02X) not found: %s", INA226_1_ADDR, esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "PUMP (INA1, 0x%02X) initialized", INA226_1_ADDR);
    }

    err = ina226_init(INA226_2_ADDR, INA226_LOGIC_SHUNT_RESISTANCE_OHM);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "LOGIC (INA2, 0x%02X) not found: %s", INA226_2_ADDR, esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "LOGIC (INA2, 0x%02X) initialized", INA226_2_ADDR);
    }

    ble_init();
    ESP_LOGI(TAG, "BLE initialized");

    err = pump_operation_set_duty_percent((uint8_t)PUMP_START_PERCENT);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set initial pump speed: %s", esp_err_to_name(err));
        return;
    }
    current_pump_percent = PUMP_START_PERCENT;
    update_indicator_leds(current_pump_percent);

    ESP_LOGI(TAG, "Pump initialized to %lu%%", (unsigned long)PUMP_START_PERCENT);
    ESP_LOGI(TAG, "System ready. Control via buttons:");
    ESP_LOGI(TAG, "  D2 (GPIO5): Toggle between MANUAL and AUTOMATIC modes");
    ESP_LOGI(TAG, "  A2/A3 (GPIO3/4): Manual mode - Increase/decrease pump (+/- 10%%)");
    ESP_LOGI(TAG, "  MANUAL mode: Pump level shown on D9/D8/D7 LEDs");
    ESP_LOGI(TAG, "  AUTOMATIC mode: Pump controlled by temperature, animation on D5-D9");

    while (1) {
        const int poll_loops = SENSOR_READ_INTERVAL_MS / BUTTON_POLL_INTERVAL_MS;

        for (int i = 0; i < poll_loops; i++) {
            uint32_t now_ms = esp_log_timestamp();

            update_status_leds(now_ms);
            update_flash_leds(now_ms);

            if (button_pressed(&button_d2)) {
                toggle_led();
                anim_led_index = 0;
                last_anim_update_ms = now_ms;
                status_led_state = false;
                last_status_led_toggle_ms = now_ms;

                if (control_mode == CONTROL_MANUAL) {
                    gpio_set_level(LED_ANIM_D5, 0);
                    gpio_set_level(LED_ANIM_D6, 0);
                    gpio_set_level(LED_ANIM_D7, 0);
                    gpio_set_level(LED_ANIM_D8, 0);
                    gpio_set_level(LED_ANIM_D9, 0);
                }
            }

            if (control_mode == CONTROL_MANUAL) {
                if (button_pressed(&button_a2)) {
                    uint32_t new_percent = current_pump_percent + PUMP_INCREMENT;
                    if (new_percent > 100) {
                        new_percent = 100;
                    }
                    current_pump_percent = new_percent;
                    refresh_pump_output_state();
                    ESP_LOGI(TAG, "Pump increased to %lu%%", (unsigned long)new_percent);
                }

                if (button_pressed(&button_a3)) {
                    int32_t new_percent = (int32_t)current_pump_percent - PUMP_INCREMENT;
                    if (new_percent < 0) {
                        new_percent = 0;
                    }
                    current_pump_percent = (uint32_t)new_percent;
                    refresh_pump_output_state();
                    ESP_LOGI(TAG, "Pump decreased to %lu%%", (unsigned long)new_percent);
                }
            } else {
                update_animation_leds(now_ms);
            }

            vTaskDelay(pdMS_TO_TICKS(BUTTON_POLL_INTERVAL_MS));
        }

        float temp_c = 0.0f;
        ina226_measurement_t ina1_measurement = synchronized_ina1_measurement;
        ina226_measurement_t ina2_measurement = synchronized_ina2_measurement;
        esp_err_t ina1_err = synchronized_ina1_error;
        esp_err_t ina2_err = synchronized_ina2_error;

        err = tmp117_read_temperature_c(TMP117_ADDR, &temp_c);
        float temp_f = (temp_c * 9.0f / 5.0f) + 32.0f;

        const float alpha = 0.2f;

        if (ina1_err == ESP_OK) {
            float curr_ma = ina1_measurement.current_a * 1000.0f;
            float pow_mw = ina1_measurement.power_w * 1000.0f;
            if (!filter_ina1_initialized) {
                filtered_ina1_current_ma = curr_ma;
                filtered_ina1_power_mw = pow_mw;
                filter_ina1_initialized = true;
            } else {
                filtered_ina1_current_ma = alpha * curr_ma + (1.0f - alpha) * filtered_ina1_current_ma;
                filtered_ina1_power_mw = alpha * pow_mw + (1.0f - alpha) * filtered_ina1_power_mw;
            }
        }

        if (ina2_err == ESP_OK) {
            float curr_ma = ina2_measurement.current_a * 1000.0f;
            float pow_mw = ina2_measurement.power_w * 1000.0f;
            if (!filter_ina2_initialized) {
                filtered_ina2_current_ma = curr_ma;
                filtered_ina2_power_mw = pow_mw;
                filter_ina2_initialized = true;
            } else {
                filtered_ina2_current_ma = alpha * curr_ma + (1.0f - alpha) * filtered_ina2_current_ma;
                filtered_ina2_power_mw = alpha * pow_mw + (1.0f - alpha) * filtered_ina2_power_mw;
            }
        }

        if (err == ESP_OK) {
            if (ina1_err == ESP_OK && ina2_err == ESP_OK) {
                ESP_LOGI(
                    TAG,
                    "Pump PWM: %lu%% | TMP117: %.2f°C / %.2f°F\n"
                    "  PUMP : BUS=%.3fV  SHUNT=%.3fmV  CURRENT=%.3fmA  POWER=%.3fmW (smoothed)\n"
                    "  LOGIC: BUS=%.3fV  SHUNT=%.3fmV  CURRENT=%.3fmA  POWER=%.3fmW (smoothed)",
                    (unsigned long)current_pump_percent,
                    temp_c,
                    temp_f,
                    ina1_measurement.bus_voltage_v,
                    ina1_measurement.shunt_voltage_v * 1000.0f,
                    filtered_ina1_current_ma,
                    filtered_ina1_power_mw,
                    ina2_measurement.bus_voltage_v,
                    ina2_measurement.shunt_voltage_v * 1000.0f,
                    filtered_ina2_current_ma,
                    filtered_ina2_power_mw);

                ESP_LOGI(TAG, "PUMP  0x%02X raw shunt=0x%04X raw bus=0x%04X",
                         INA226_1_ADDR, ina1_measurement.raw_shunt_u16, ina1_measurement.raw_bus_u16);
                ESP_LOGI(TAG, "LOGIC 0x%02X raw shunt=0x%04X raw bus=0x%04X",
                         INA226_2_ADDR, ina2_measurement.raw_shunt_u16, ina2_measurement.raw_bus_u16);

                if (ble_is_connected()) {
                    char msg[160];
                    snprintf(
                        msg,
                        sizeof(msg),
                        "TempF=%.2f,Pump=%lu,INA1=%.2fmA,INA2=%.2fmA",
                        temp_f,
                        (unsigned long)current_pump_percent,
                        filtered_ina1_current_ma,
                        filtered_ina2_current_ma);
                    ble_notify(msg);
                }

            } else if (ina1_err == ESP_OK) {
                ESP_LOGI(
                    TAG,
                    "Pump PWM: %lu%% | TMP117: %.2f°C / %.2f°F\n"
                    "  PUMP : BUS=%.3fV  SHUNT=%.3fmV  CURRENT=%.3fmA  POWER=%.3fmW (smoothed)",
                    (unsigned long)current_pump_percent,
                    temp_c,
                    temp_f,
                    ina1_measurement.bus_voltage_v,
                    ina1_measurement.shunt_voltage_v * 1000.0f,
                    filtered_ina1_current_ma,
                    filtered_ina1_power_mw);

                ESP_LOGI(TAG, "PUMP  0x%02X raw shunt=0x%04X raw bus=0x%04X",
                         INA226_1_ADDR, ina1_measurement.raw_shunt_u16, ina1_measurement.raw_bus_u16);
                ESP_LOGW(TAG, "INA2 read failed: %s", esp_err_to_name(ina2_err));

            } else if (ina2_err == ESP_OK) {
                ESP_LOGI(
                    TAG,
                    "Pump PWM: %lu%% | TMP117: %.2f°C / %.2f°F\n"
                    "  LOGIC: BUS=%.3fV  SHUNT=%.3fmV  CURRENT=%.3fmA  POWER=%.3fmW (smoothed)",
                    (unsigned long)current_pump_percent,
                    temp_c,
                    temp_f,
                    ina2_measurement.bus_voltage_v,
                    ina2_measurement.shunt_voltage_v * 1000.0f,
                    filtered_ina2_current_ma,
                    filtered_ina2_power_mw);

                ESP_LOGI(TAG, "LOGIC 0x%02X raw shunt=0x%04X raw bus=0x%04X",
                         INA226_2_ADDR, ina2_measurement.raw_shunt_u16, ina2_measurement.raw_bus_u16);
                ESP_LOGW(TAG, "INA1 read failed: %s", esp_err_to_name(ina1_err));

            } else {
                ESP_LOGI(TAG, "Pump PWM: %lu%% | TMP117: %.2f°C / %.2f°F",
                         (unsigned long)current_pump_percent, temp_c, temp_f);
                ESP_LOGW(TAG, "PUMP read failed: %s", esp_err_to_name(ina1_err));
                ESP_LOGW(TAG, "LOGIC read failed: %s", esp_err_to_name(ina2_err));
            }

            if (control_mode == CONTROL_AUTOMATIC) {
                update_pump_auto(temp_f);
            }

        } else {
            ESP_LOGW(TAG, "TMP117 read failed: %s", esp_err_to_name(err));

            if (ina1_err == ESP_OK) {
                ESP_LOGI(
                    TAG,
                    "Pump PWM: %lu%% | TMP117: (read failed)\n"
                    "  PUMP : BUS=%.3fV  SHUNT=%.3fmV  CURRENT=%.3fmA  POWER=%.3fmW (smoothed)",
                    (unsigned long)current_pump_percent,
                    ina1_measurement.bus_voltage_v,
                    ina1_measurement.shunt_voltage_v * 1000.0f,
                    filtered_ina1_current_ma,
                    filtered_ina1_power_mw);
                ESP_LOGI(TAG, "PUMP  0x%02X raw shunt=0x%04X raw bus=0x%04X",
                         INA226_1_ADDR, ina1_measurement.raw_shunt_u16, ina1_measurement.raw_bus_u16);
            } else {
                ESP_LOGW(TAG, "PUMP read failed: %s", esp_err_to_name(ina1_err));
            }

            if (ina2_err == ESP_OK) {
                ESP_LOGI(
                    TAG,
                    "Pump PWM: %lu%% | TMP117: (read failed)\n"
                    "  LOGIC: BUS=%.3fV  SHUNT=%.3fmV  CURRENT=%.3fmA  POWER=%.3fmW (smoothed)",
                    (unsigned long)current_pump_percent,
                    ina2_measurement.bus_voltage_v,
                    ina2_measurement.shunt_voltage_v * 1000.0f,
                    filtered_ina2_current_ma,
                    filtered_ina2_power_mw);
                ESP_LOGI(TAG, "LOGIC 0x%02X raw shunt=0x%04X raw bus=0x%04X",
                         INA226_2_ADDR, ina2_measurement.raw_shunt_u16, ina2_measurement.raw_bus_u16);
            } else {
                ESP_LOGW(TAG, "LOGIC read failed: %s", esp_err_to_name(ina2_err));
            }
        }
    }
}