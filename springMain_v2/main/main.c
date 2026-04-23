#include <stdio.h>
#include <stdbool.h>

#include "esp_err.h"
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
static bool s_ina1_present = false;
static bool s_ina2_present = false;
static uint8_t s_ina1_fail_count = 0;
static uint8_t s_ina2_fail_count = 0;
static uint32_t s_ina1_next_retry_ms = 0;
static uint32_t s_ina2_next_retry_ms = 0;
static uint8_t s_tmp117_addr = 0x48;
static bool s_tmp117_found = false;
static bool s_tmp117_primary_found = false;
static bool s_tmp117_secondary_found = false;
static bool s_tmp117_tertiary_found = false;

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

#define PUMP_INCREMENT 25           // 25% per button press
#define PUMP_START_PERCENT 50       // Starting pump intensity

// I2C Sensor Addresses
#define INA226_1_ADDR 0x40
#define INA226_2_ADDR 0x41
#define INA226_PUMP_ADDR INA226_1_ADDR
#define INA226_LOGIC_ADDR INA226_2_ADDR
#define TMP117_ADDR_PRIMARY 0x48
#define TMP117_ADDR_SECONDARY 0x49
#define TMP117_ADDR_TERTIARY 0x50

#if (INA226_PUMP_ADDR == INA226_LOGIC_ADDR)
#error "INA226_PUMP_ADDR and INA226_LOGIC_ADDR must be different"
#endif

// INA226 hardware configuration
#define INA226_PUMP_SHUNT_RESISTANCE_OHM 0.1f
#define INA226_LOGIC_SHUNT_RESISTANCE_OHM 0.1f

// Timing configuration
#define BUTTON_POLL_INTERVAL_MS 50
#define SENSOR_READ_INTERVAL_MS 2000

// INA synchronized reading configuration
#define INA_SYNC_PERIOD_MS 17
#define INA_READ_OFFSET_MS 3
#define INA_READ_PHASE_DITHER_MS 7
#define INA_READING_TASK_PRIORITY (tskIDLE_PRIORITY + 2)
#define INA_READING_TASK_STACK_SIZE 4096
#define DEVICE_FAILURE_DISABLE_THRESHOLD 5
#define DEVICE_RETRY_INTERVAL_MS 5000

// Temporary deep diagnostics for I2C bring-up.
#define I2C_DIAG_START_ADDR 0x03
#define I2C_DIAG_END_ADDR 0x77
#define I2C_ENABLE_FULL_DIAGNOSTICS 0
#define I2C_DIAG_YIELD_EVERY 8

typedef enum {
    CONTROL_MANUAL = 0,
    CONTROL_AUTOMATIC = 1
} control_mode_t;

// Temperature thresholds for automatic control (Fahrenheit)
#define TEMP_STEP_98F 98.0f
#define TEMP_STEP_99F 99.0f
#define TEMP_STEP_99_5F 99.5f
#define TEMP_STEP_100F 100.0f
#define TEMP_SHUTDOWN_108F 101.0f

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
static void update_animation_leds(uint32_t current_ms);
static void update_pump_level_leds(uint32_t pump_percent);
static void update_mode_led(uint32_t current_ms);
static void refresh_pump_output_state(void);
static void verify_ina_address_mapping(void);

#if I2C_ENABLE_FULL_DIAGNOSTICS
static void log_i2c_probe_diagnostics(TickType_t timeout_ticks)
{
    ESP_LOGI(TAG, "I2C diagnostics: probing 0x%02X..0x%02X", I2C_DIAG_START_ADDR, I2C_DIAG_END_ADDR);
    uint8_t probe_count = 0;

    for (uint8_t addr = I2C_DIAG_START_ADDR; addr <= I2C_DIAG_END_ADDR; ++addr) {
        esp_err_t perr = i2c_bus_probe(addr, timeout_ticks);
        if (perr == ESP_OK) {
            ESP_LOGI(TAG, "I2C probe 0x%02X: ACK", addr);
        } else {
            ESP_LOGW(TAG, "I2C probe 0x%02X: %s", addr, esp_err_to_name(perr));
        }

        probe_count++;
        if ((probe_count % I2C_DIAG_YIELD_EVERY) == 0) {
            // Yield during long probe sweeps to avoid startup watchdog trips.
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
}
#endif

// PWM-synchronized INA reading callback
static void pwm_sync_timer_callback(void *arg)
{
    // Timer uses ESP_TIMER_TASK dispatch, so this callback is not in ISR context.
    if (ina_read_semaphore != NULL) {
        xSemaphoreGive(ina_read_semaphore);
    }
}

// Dedicated task for synchronized INA readings
static void ina_reading_task(void *arg)
{
    uint32_t sample_count = 0;

    while (1) {
        if (xSemaphoreTake(ina_read_semaphore, portMAX_DELAY) == pdTRUE) {
            uint32_t now_ms = esp_log_timestamp();

            // Dither the read phase so INA samples do not lock to a fixed PWM phase.
            uint32_t phase_dither_ms = sample_count % INA_READ_PHASE_DITHER_MS;
            vTaskDelay(pdMS_TO_TICKS(INA_READ_OFFSET_MS + phase_dither_ms));
            sample_count++;

            if (s_ina1_present) {
                synchronized_ina1_error = ina226_read_measurement(
                    INA226_1_ADDR,
                    INA226_PUMP_SHUNT_RESISTANCE_OHM,
                    &synchronized_ina1_measurement);

                if (synchronized_ina1_error == ESP_OK) {
                    s_ina1_fail_count = 0;
                } else {
                    if (s_ina1_fail_count < 255) {
                        s_ina1_fail_count++;
                    }
                    if (s_ina1_fail_count >= DEVICE_FAILURE_DISABLE_THRESHOLD) {
                        s_ina1_present = false;
                        s_ina1_next_retry_ms = now_ms + DEVICE_RETRY_INTERVAL_MS;
                        ESP_LOGW(TAG,
                                 "INA1 (0x%02X) marked offline after %u read failures; retry in %u ms",
                                 INA226_1_ADDR,
                                 (unsigned)s_ina1_fail_count,
                                 (unsigned)DEVICE_RETRY_INTERVAL_MS);
                    }
                }
            } else if (now_ms >= s_ina1_next_retry_ms) {
                esp_err_t reinit_err = ina226_init(INA226_1_ADDR, INA226_PUMP_SHUNT_RESISTANCE_OHM);
                if (reinit_err == ESP_OK) {
                    s_ina1_present = true;
                    s_ina1_fail_count = 0;
                    synchronized_ina1_error = ESP_OK;
                    ESP_LOGI(TAG, "INA1 (0x%02X) recovered and re-enabled", INA226_1_ADDR);
                } else {
                    synchronized_ina1_error = reinit_err;
                    s_ina1_next_retry_ms = now_ms + DEVICE_RETRY_INTERVAL_MS;
                }
            } else {
                synchronized_ina1_error = ESP_ERR_NOT_FOUND;
            }

            if (s_ina2_present) {
                synchronized_ina2_error = ina226_read_measurement(
                    INA226_2_ADDR,
                    INA226_LOGIC_SHUNT_RESISTANCE_OHM,
                    &synchronized_ina2_measurement);

                if (synchronized_ina2_error == ESP_OK) {
                    s_ina2_fail_count = 0;
                } else {
                    if (s_ina2_fail_count < 255) {
                        s_ina2_fail_count++;
                    }
                    if (s_ina2_fail_count >= DEVICE_FAILURE_DISABLE_THRESHOLD) {
                        s_ina2_present = false;
                        s_ina2_next_retry_ms = now_ms + DEVICE_RETRY_INTERVAL_MS;
                        ESP_LOGW(TAG,
                                 "INA2 (0x%02X) marked offline after %u read failures; retry in %u ms",
                                 INA226_2_ADDR,
                                 (unsigned)s_ina2_fail_count,
                                 (unsigned)DEVICE_RETRY_INTERVAL_MS);
                    }
                }
            } else if (now_ms >= s_ina2_next_retry_ms) {
                esp_err_t reinit_err = ina226_init(INA226_2_ADDR, INA226_LOGIC_SHUNT_RESISTANCE_OHM);
                if (reinit_err == ESP_OK) {
                    s_ina2_present = true;
                    s_ina2_fail_count = 0;
                    synchronized_ina2_error = ESP_OK;
                    ESP_LOGI(TAG, "INA2 (0x%02X) recovered and re-enabled", INA226_2_ADDR);
                } else {
                    synchronized_ina2_error = reinit_err;
                    s_ina2_next_retry_ms = now_ms + DEVICE_RETRY_INTERVAL_MS;
                }
            } else {
                synchronized_ina2_error = ESP_ERR_NOT_FOUND;
            }

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
static uint8_t pump_no_current_streak = 0;
static button_state_t button_d2 = {BUTTON_D2, true, true};
static button_state_t button_a2 = {BUTTON_A2, true, true};
static button_state_t button_a3 = {BUTTON_A3, true, true};
static esp_timer_handle_t pwm_sync_timer = NULL;

// Battery countdown timer (3 minutes = 180 seconds)
#define BATTERY_COUNTDOWN_SECONDS 180
static uint32_t battery_timer_start_ms = 0;
static bool battery_timer_active = true;
static uint32_t last_battery_blink_ms = 0;
static bool battery_blink_state = false;

static void toggle_led(void)
{
    control_mode = (control_mode == CONTROL_MANUAL) ? CONTROL_AUTOMATIC : CONTROL_MANUAL;

    refresh_pump_output_state();

    if (control_mode == CONTROL_MANUAL) {
        update_pump_level_leds(current_pump_percent);
        ESP_LOGI(TAG, "Control mode: MANUAL - Use A2/A3 buttons to adjust pump");
    } else {
        ESP_LOGI(TAG, "Control mode: AUTOMATIC - Pump controlled by temperature");
    }
}


static void update_pump_auto(float temp_f)
{
    uint32_t new_pump_percent = current_pump_percent;

    // Emergency shutdown if temperature exceeds 101°F
    if (temp_f > TEMP_SHUTDOWN_108F) {
        new_pump_percent = 0;
        if (current_pump_percent != 0) {
            current_pump_percent = 0;
            refresh_pump_output_state();
            ESP_LOGW(TAG, "EMERGENCY SHUTDOWN: Temperature %.2f°F exceeds 101°F limit - Pump OFF",
                     temp_f);
        }
        return;
    }

    if (temp_f <= TEMP_STEP_98F) {
        new_pump_percent = 0;
    }
    else if (temp_f < TEMP_STEP_99F) {
        new_pump_percent = 0;
    }
    else if (temp_f < TEMP_STEP_99_5F) {
        new_pump_percent = PUMP_25_PERCENT;
    }
    else if (temp_f < TEMP_STEP_100F) {
        new_pump_percent = PUMP_50_PERCENT;
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
    // Battery countdown display (D3, D4, D5)
    // Counts down 3 minutes: 3min=all on, 2min=2 on, 1min=1 blinking, 0=all off
    if (battery_timer_active) {
        uint32_t elapsed_ms = current_ms - battery_timer_start_ms;
        uint32_t elapsed_s = elapsed_ms / 1000;
        uint32_t remaining_s = (elapsed_s < BATTERY_COUNTDOWN_SECONDS) 
                                ? (BATTERY_COUNTDOWN_SECONDS - elapsed_s) 
                                : 0;
        
        // Log battery status every 60 seconds (at transitions)
        static uint32_t last_battery_log_s = BATTERY_COUNTDOWN_SECONDS;
        uint32_t current_minute = remaining_s / 60;
        if (current_minute != last_battery_log_s / 60) {
            last_battery_log_s = remaining_s;
            if (remaining_s > 0) {
                ESP_LOGI(TAG, "Battery: %lu minutes remaining", remaining_s / 60);
            } else {
                ESP_LOGI(TAG, "Battery: DEPLETED");
                battery_timer_active = false;
            }
        }
        
        if (remaining_s > 120) {
            // 3 min range: all 3 LEDs on
            gpio_set_level(LED_AUTO_STATUS_1, 1);  // D3
            gpio_set_level(LED_AUTO_STATUS_2, 1);  // D4
            gpio_set_level(LED_ANIM_D5, 1);        // D5
        } else if (remaining_s > 60) {
            // 2 min range: 2 LEDs on
            gpio_set_level(LED_AUTO_STATUS_1, 1);  // D3
            gpio_set_level(LED_AUTO_STATUS_2, 1);  // D4
            gpio_set_level(LED_ANIM_D5, 0);        // D5
        } else if (remaining_s > 0) {
            // 1 min range: 1 LED blinking
            gpio_set_level(LED_AUTO_STATUS_2, 0);  // D4
            if (current_ms - last_battery_blink_ms >= LED_FLASH_INTERVAL_MS) {
                battery_blink_state = !battery_blink_state;
                last_battery_blink_ms = current_ms;
            }
            gpio_set_level(LED_AUTO_STATUS_1, battery_blink_state ? 1 : 0);  // D3 blinking
        } else {
            // Time expired: all off
            gpio_set_level(LED_AUTO_STATUS_1, 0);  // D3
            gpio_set_level(LED_AUTO_STATUS_2, 0);  // D4
            gpio_set_level(LED_ANIM_D5, 0);        // D5
        }
    }
}

static void update_pump_level_leds(uint32_t pump_percent)
{
    // Pump level indicator (D6, D7, D8)
    // 0%   = all off
    // 1-33%  = D6 only
    // 34-66% = D6 + D7
    // 67-100% = D6 + D7 + D8
    if (pump_percent == 0) {
        gpio_set_level(LED_ANIM_D6, 0);
        gpio_set_level(LED_ANIM_D7, 0);
        gpio_set_level(LED_ANIM_D8, 0);
    } else if (pump_percent <= 33) {
        gpio_set_level(LED_ANIM_D6, 1);
        gpio_set_level(LED_ANIM_D7, 0);
        gpio_set_level(LED_ANIM_D8, 0);
    } else if (pump_percent <= 66) {
        gpio_set_level(LED_ANIM_D6, 1);
        gpio_set_level(LED_ANIM_D7, 1);
        gpio_set_level(LED_ANIM_D8, 0);
    } else {
        gpio_set_level(LED_ANIM_D6, 1);
        gpio_set_level(LED_ANIM_D7, 1);
        gpio_set_level(LED_ANIM_D8, 1);
    }
}

static void update_mode_led(uint32_t current_ms)
{
    // Mode indicator (D9)
    // Manual mode: flashing
    // Auto mode: constant on
    if (control_mode == CONTROL_MANUAL) {
        if (current_ms - last_flash_toggle_ms >= LED_FLASH_INTERVAL_MS) {
            flash_state = !flash_state;
            last_flash_toggle_ms = current_ms;
        }
        gpio_set_level(LED_ANIM_D9, flash_state ? 1 : 0);
    } else {
        gpio_set_level(LED_ANIM_D9, 1);  // Constant on in auto mode
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

    uint64_t timer_period_us = (INA_SYNC_PERIOD_MS * 1000);
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
        update_pump_level_leds(current_pump_percent);
    }
}

static void verify_ina_address_mapping(void)
{
    esp_err_t pump_probe = i2c_bus_probe(INA226_PUMP_ADDR, pdMS_TO_TICKS(10));
    esp_err_t logic_probe = i2c_bus_probe(INA226_LOGIC_ADDR, pdMS_TO_TICKS(10));

    if (pump_probe == ESP_OK && logic_probe == ESP_OK) {
        ESP_LOGI(TAG,
                 "INA address map verified: PUMP=0x%02X, LOGIC=0x%02X",
                 INA226_PUMP_ADDR,
                 INA226_LOGIC_ADDR);
        return;
    }

    ESP_LOGW(TAG,
             "INA address map check: expected PUMP=0x%02X (%s), LOGIC=0x%02X (%s)",
             INA226_PUMP_ADDR,
             esp_err_to_name(pump_probe),
             INA226_LOGIC_ADDR,
             esp_err_to_name(logic_probe));
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
    i2c_bus_scan(pdMS_TO_TICKS(10));
    verify_ina_address_mapping();

    err = pump_operation_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize pump output: %s", esp_err_to_name(err));
        return;
    }

    init_buttons_and_led();

    // Initialize battery countdown timer (3 minutes)
    battery_timer_start_ms = esp_timer_get_time() / 1000;
    ESP_LOGI(TAG, "Battery timer: 3 minutes countdown started");

    const uint8_t tmp117_candidates[] = {TMP117_ADDR_PRIMARY, TMP117_ADDR_SECONDARY, TMP117_ADDR_TERTIARY};
    const char *tmp117_labels[] = {"BODY (0x48)", "BODY (0x49)", "ENVIRONMENT (0x50)"};
    for (size_t i = 0; i < sizeof(tmp117_candidates) / sizeof(tmp117_candidates[0]); ++i) {
        esp_err_t terr = tmp117_init(tmp117_candidates[i]);
        if (terr == ESP_OK) {
            if (tmp117_candidates[i] == TMP117_ADDR_PRIMARY) {
                s_tmp117_primary_found = true;
            }
            if (tmp117_candidates[i] == TMP117_ADDR_SECONDARY) {
                s_tmp117_secondary_found = true;
            }
            if (tmp117_candidates[i] == TMP117_ADDR_TERTIARY) {
                s_tmp117_tertiary_found = true;
            }
            if (!s_tmp117_found) {
                s_tmp117_addr = tmp117_candidates[i];
                s_tmp117_found = true;
            }
            ESP_LOGI(TAG, "TMP117 %s (0x%02X): FOUND", tmp117_labels[i], tmp117_candidates[i]);
        } else {
            ESP_LOGW(TAG, "TMP117 %s (0x%02X): NOT FOUND", tmp117_labels[i], tmp117_candidates[i]);
        }
    }
    if (s_tmp117_found) {
        ESP_LOGI(TAG, "TMP117 selected address: 0x%02X", s_tmp117_addr);
    }
    if (!s_tmp117_found) {
        ESP_LOGE(TAG,
                 "TMP117 not found at expected addresses (0x%02X, 0x%02X, 0x%02X)",
                 TMP117_ADDR_PRIMARY,
                 TMP117_ADDR_SECONDARY,
                 TMP117_ADDR_TERTIARY);
    }

    err = ina226_init(INA226_PUMP_ADDR, INA226_PUMP_SHUNT_RESISTANCE_OHM);
    if (err != ESP_OK) {
        s_ina1_present = false;
        s_ina1_next_retry_ms = esp_log_timestamp() + DEVICE_RETRY_INTERVAL_MS;
        synchronized_ina1_error = err;
        ESP_LOGW(TAG, "PUMP (INA1, 0x%02X) not found: %s", INA226_PUMP_ADDR, esp_err_to_name(err));
    } else {
        s_ina1_present = true;
        s_ina1_fail_count = 0;
        synchronized_ina1_error = ESP_OK;
        ESP_LOGI(TAG, "PUMP (INA1, 0x%02X) initialized", INA226_PUMP_ADDR);
    }

    err = ina226_init(INA226_LOGIC_ADDR, INA226_LOGIC_SHUNT_RESISTANCE_OHM);
    if (err != ESP_OK) {
        s_ina2_present = false;
        s_ina2_next_retry_ms = esp_log_timestamp() + DEVICE_RETRY_INTERVAL_MS;
        synchronized_ina2_error = err;
        ESP_LOGW(TAG, "LOGIC (INA2, 0x%02X) not found: %s", INA226_LOGIC_ADDR, esp_err_to_name(err));
    } else {
        s_ina2_present = true;
        s_ina2_fail_count = 0;
        synchronized_ina2_error = ESP_OK;
        ESP_LOGI(TAG, "LOGIC (INA2, 0x%02X) initialized", INA226_LOGIC_ADDR);
    }

    // Full-range diagnostics are expensive; keep disabled in production boot path.
#if I2C_ENABLE_FULL_DIAGNOSTICS
    log_i2c_probe_diagnostics(pdMS_TO_TICKS(10));
#endif

    init_ina_sync_reading();

    ble_init();
    ESP_LOGI(TAG, "BLE initialized");

    err = pump_operation_set_duty_percent((uint8_t)PUMP_START_PERCENT);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set initial pump speed: %s", esp_err_to_name(err));
        return;
    }
    current_pump_percent = PUMP_START_PERCENT;
    update_pump_level_leds(current_pump_percent);

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

            // Update mode indicator first (always)
            update_mode_led(now_ms);              // Mode indicator (D9)

            // Update mode-specific LED indicators
            if (control_mode == CONTROL_MANUAL) {
                update_pump_level_leds(current_pump_percent);  // Pump level (D6, D7, D8)
            } else {
                update_animation_leds(now_ms);    // Battery countdown (D3, D4, D5)
            }

            if (button_pressed(&button_d2)) {
                toggle_led();
                anim_led_index = 0;
                last_anim_update_ms = now_ms;
                status_led_state = false;
                last_status_led_toggle_ms = now_ms;
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
            }

            vTaskDelay(pdMS_TO_TICKS(BUTTON_POLL_INTERVAL_MS));
        }

        float temp_c = 0.0f;
        ina226_measurement_t ina1_measurement = synchronized_ina1_measurement;
        ina226_measurement_t ina2_measurement = synchronized_ina2_measurement;
        esp_err_t ina1_err = synchronized_ina1_error;
        esp_err_t ina2_err = synchronized_ina2_error;

        if (s_tmp117_found) {
            err = tmp117_read_temperature_c(s_tmp117_addr, &temp_c);
        } else {
            err = ESP_ERR_NOT_FOUND;
        }
        float temp_f = (temp_c * 9.0f / 5.0f) + 32.0f;

        float temp_primary_c = 0.0f;
        float temp_secondary_c = 0.0f;
        float temp_tertiary_c = 0.0f;
        esp_err_t tmp_primary_err = s_tmp117_primary_found
                                        ? tmp117_read_temperature_c(TMP117_ADDR_PRIMARY, &temp_primary_c)
                                        : ESP_ERR_NOT_FOUND;
        esp_err_t tmp_secondary_err = s_tmp117_secondary_found
                                          ? tmp117_read_temperature_c(TMP117_ADDR_SECONDARY, &temp_secondary_c)
                                          : ESP_ERR_NOT_FOUND;
        esp_err_t tmp_tertiary_err = s_tmp117_tertiary_found
                                         ? tmp117_read_temperature_c(TMP117_ADDR_TERTIARY, &temp_tertiary_c)
                                         : ESP_ERR_NOT_FOUND;
        float temp_primary_f = (temp_primary_c * 9.0f / 5.0f) + 32.0f;
        float temp_secondary_f = (temp_secondary_c * 9.0f / 5.0f) + 32.0f;
        float temp_tertiary_f = (temp_tertiary_c * 9.0f / 5.0f) + 32.0f;

        char tmp_primary_str[32];
        char tmp_secondary_str[32];
        char tmp_tertiary_str[32];
        if (tmp_primary_err == ESP_OK) {
            snprintf(tmp_primary_str, sizeof(tmp_primary_str), "%.2fC/%.2fF", temp_primary_c, temp_primary_f);
        } else {
            snprintf(tmp_primary_str, sizeof(tmp_primary_str), "N/A");
        }
        if (tmp_secondary_err == ESP_OK) {
            snprintf(tmp_secondary_str, sizeof(tmp_secondary_str), "%.2fC/%.2fF", temp_secondary_c, temp_secondary_f);
        } else {
            snprintf(tmp_secondary_str, sizeof(tmp_secondary_str), "N/A");
        }
        if (tmp_tertiary_err == ESP_OK) {
            snprintf(tmp_tertiary_str, sizeof(tmp_tertiary_str), "%.2fC/%.2fF", temp_tertiary_c, temp_tertiary_f);
        } else {
            snprintf(tmp_tertiary_str, sizeof(tmp_tertiary_str), "N/A");
        }
        ESP_LOGI(TAG,
                 "TMP117 BODY (0x%02X): %s | BODY (0x%02X): %s | ENVIRONMENT (0x%02X): %s",
                 TMP117_ADDR_PRIMARY,
                 tmp_primary_str,
                 TMP117_ADDR_SECONDARY,
                 tmp_secondary_str,
                 TMP117_ADDR_TERTIARY,
                 tmp_tertiary_str);

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

            if (current_pump_percent >= 50 && filtered_ina1_current_ma < 2.0f && filtered_ina1_current_ma > -2.0f) {
                if (pump_no_current_streak < 255) {
                    pump_no_current_streak++;
                }
            } else {
                pump_no_current_streak = 0;
            }

            if (pump_no_current_streak == 5) {
                ESP_LOGW(
                    TAG,
                    "Pump PWM >= 50%% but INA1 current remains near zero; check shunt path/wiring polarity (INA1 0x%02X)",
                    INA226_1_ADDR);
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
            if (ina1_err == ESP_OK) {
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

                if (ina2_err == ESP_OK) {
                    ESP_LOGI(
                        TAG,
                        "  LOGIC: BUS=%.3fV  SHUNT=%.3fmV  CURRENT=%.3fmA  POWER=%.3fmW (smoothed)",
                        ina2_measurement.bus_voltage_v,
                        ina2_measurement.shunt_voltage_v * 1000.0f,
                        filtered_ina2_current_ma,
                        filtered_ina2_power_mw);
                    ESP_LOGI(TAG, "LOGIC 0x%02X raw shunt=0x%04X raw bus=0x%04X",
                             INA226_2_ADDR, ina2_measurement.raw_shunt_u16, ina2_measurement.raw_bus_u16);
                } else {
                    ESP_LOGW(TAG, "LOGIC read failed: %s", esp_err_to_name(ina2_err));
                }

            } else {
                ESP_LOGI(TAG, "Pump PWM: %lu%% | TMP117: %.2f°C / %.2f°F",
                         (unsigned long)current_pump_percent, temp_c, temp_f);
                ESP_LOGW(TAG, "PUMP read failed: %s", esp_err_to_name(ina1_err));
                if (ina2_err == ESP_OK) {
                    ESP_LOGI(
                        TAG,
                        "  LOGIC: BUS=%.3fV  SHUNT=%.3fmV  CURRENT=%.3fmA  POWER=%.3fmW (smoothed)",
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

                if (ina2_err == ESP_OK) {
                    ESP_LOGI(
                        TAG,
                        "  LOGIC: BUS=%.3fV  SHUNT=%.3fmV  CURRENT=%.3fmA  POWER=%.3fmW (smoothed)",
                        ina2_measurement.bus_voltage_v,
                        ina2_measurement.shunt_voltage_v * 1000.0f,
                        filtered_ina2_current_ma,
                        filtered_ina2_power_mw);
                    ESP_LOGI(TAG, "LOGIC 0x%02X raw shunt=0x%04X raw bus=0x%04X",
                             INA226_2_ADDR, ina2_measurement.raw_shunt_u16, ina2_measurement.raw_bus_u16);
                } else {
                    ESP_LOGW(TAG, "LOGIC read failed: %s", esp_err_to_name(ina2_err));
                }
            } else {
                ESP_LOGW(TAG, "PUMP read failed: %s", esp_err_to_name(ina1_err));
                if (ina2_err == ESP_OK) {
                    ESP_LOGI(
                        TAG,
                        "  LOGIC: BUS=%.3fV  SHUNT=%.3fmV  CURRENT=%.3fmA  POWER=%.3fmW (smoothed)",
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
}