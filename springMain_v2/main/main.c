#include <stdbool.h>

#include "esp_log.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "ina226.h"
#include "tmp117.h"
#include "i2c_bus.h"
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

// PWM Configuration
#define PWM_LINE_GPIO GPIO_NUM_1   // Nano ESP32 A0 = GPIO1
#define PWM_TIMER LEDC_TIMER_0
#define PWM_MODE LEDC_LOW_SPEED_MODE
#define PWM_CHANNEL LEDC_CHANNEL_0
#define PWM_RESOLUTION LEDC_TIMER_10_BIT
#define PWM_FREQ_HZ 500

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
#define INA226_1_ADDR 0x40    // INA1 current/power monitor
#define INA226_2_ADDR 0x41    // INA2 current/power monitor
#define TMP117_ADDR TMP117_I2C_ADDR_DEFAULT // TMP117 at GND address 1001000x (0x48)

// INA226 hardware configuration
#define INA226_PUMP_SHUNT_RESISTANCE_OHM 0.1f    // 100 mOhm shunt (PUMP / INA1)
#define INA226_LOGIC_SHUNT_RESISTANCE_OHM 0.1f   // 100 mOhm shunt (LOGIC / INA2)

// Timing configuration
#define BUTTON_POLL_INTERVAL_MS 50
#define SENSOR_READ_INTERVAL_MS 2000

// PWM-synchronized INA reading configuration
#define PWM_PERIOD_MS (1000 / PWM_FREQ_HZ)     // 2ms at 500Hz
#define INA_READ_OFFSET_MS 3                    // Read 3ms after PWM start
#define INA_READING_TASK_PRIORITY (tskIDLE_PRIORITY + 2)
#define INA_READING_TASK_STACK_SIZE 4096

// Control mode enum
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
#define LED_ANIM_INTERVAL_MS 1000   // 1 second per LED
#define LED_FLASH_INTERVAL_MS 250   // 250ms flash for 100%

// Button state tracking for debouncing
typedef struct {
    gpio_num_t pin;
    bool stable_state;      // Last confirmed state (true=released, false=pressed)
    bool current_state;     // Current raw state
} button_state_t;

static const uint32_t PWM_MAX_DUTY = (1U << PWM_RESOLUTION) - 1U;

// Forward declarations
static void update_indicator_leds(uint32_t pump_percent);
static void update_flash_leds(uint32_t current_ms);
static void update_status_leds(uint32_t current_ms);
static void update_animation_leds(uint32_t current_ms);

// PWM-synchronized INA reading callback - fires at 500Hz (every 2ms)
static void pwm_sync_timer_callback(void *arg)
{
    // Signal the INA reading task to perform synchronized read
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
        // Wait for semaphore signal from PWM sync timer
        if (xSemaphoreTake(ina_read_semaphore, portMAX_DELAY) == pdTRUE) {
            // Delay 3ms after PWM start to allow current to stabilize
            vTaskDelay(pdMS_TO_TICKS(INA_READ_OFFSET_MS));
            
            // Read both INA sensors synchronously
            synchronized_ina1_error = ina226_read_measurement(
                INA226_1_ADDR,
                INA226_PUMP_SHUNT_RESISTANCE_OHM,
                &synchronized_ina1_measurement);
            
            synchronized_ina2_error = ina226_read_measurement(
                INA226_2_ADDR,
                INA226_LOGIC_SHUNT_RESISTANCE_OHM,
                &synchronized_ina2_measurement);
            
            // Log synchronization event (debug)
            ESP_LOGD(TAG, "INA readings synchronized at PWM cycle + 3ms");
        }
    }
}



// Global state
static uint32_t current_pump_percent = PUMP_START_PERCENT;
static control_mode_t control_mode = CONTROL_MANUAL;  // Start in manual mode
static uint32_t last_flash_toggle_ms = 0;
static bool flash_state = false;  // For 100% flashing
static uint32_t last_status_led_toggle_ms = 0;
static bool status_led_state = false;  // For status LED (D3/D4) flashing
static uint32_t last_anim_update_ms = 0;
static uint8_t anim_led_index = 0;  // Which LED in the animation sequence (0-4 for 5 LEDs)
static button_state_t button_d2 = {BUTTON_D2, true, true};
static button_state_t button_a2 = {BUTTON_A2, true, true};
static button_state_t button_a3 = {BUTTON_A3, true, true};
static esp_timer_handle_t pwm_sync_timer = NULL;

static void set_pump_percent(uint32_t pump_percent)
{
    if (pump_percent > 100U) {
        pump_percent = 100U;
    }

    uint32_t duty = (pump_percent * PWM_MAX_DUTY + 50U) / 100U;

    ledc_set_duty(PWM_MODE, PWM_CHANNEL, duty);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL);
    
    current_pump_percent = pump_percent;
    
    // Update indicator LEDs (manual mode only)
    if (control_mode == CONTROL_MANUAL) {
        update_indicator_leds(pump_percent);
    }
}

static void toggle_led(void)
{
    // D2 button toggles between manual and automatic control
    control_mode = (control_mode == CONTROL_MANUAL) ? CONTROL_AUTOMATIC : CONTROL_MANUAL;
    
    if (control_mode == CONTROL_MANUAL) {
        ESP_LOGI(TAG, "Control mode: MANUAL - Use A2/A3 buttons to adjust pump");
    } else {
        ESP_LOGI(TAG, "Control mode: AUTOMATIC - Pump controlled by temperature");
    }
}

static void update_pump_auto(float temp_f)
{
    // Automatic pump control based on Fahrenheit step thresholds.
    uint32_t new_pump_percent = current_pump_percent;
    
    if (temp_f <= TEMP_STEP_98F) {
        new_pump_percent = PUMP_10_PERCENT;      // 98F and below
    }
    else if (temp_f < TEMP_STEP_99F) {
        new_pump_percent = PUMP_10_PERCENT;      // >98F to <99F
    }
    else if (temp_f < TEMP_STEP_100F) {
        new_pump_percent = PUMP_25_PERCENT;      // 99F to <100F
    }
    else if (temp_f < TEMP_STEP_101F) {
        new_pump_percent = PUMP_50_PERCENT;      // 100F to <101F
    }
    else if (temp_f <= TEMP_STEP_102F) {
        new_pump_percent = PUMP_75_PERCENT;      // 101F to 102F
    }
    else {
        new_pump_percent = PUMP_100_PERCENT;     // >102F
    }
    
    // Update pump if temperature-based setting changed
    if (new_pump_percent != current_pump_percent) {
        set_pump_percent(new_pump_percent);
        ESP_LOGI(TAG, "Auto mode: Temp %.2f°F -> Pump %d%%", temp_f, new_pump_percent);
    }
}

static void update_animation_leds(uint32_t current_ms)
{
    // Keep D3 and D4 status LEDs on during auto mode
    gpio_set_level(LED_AUTO_STATUS_1, 1);
    gpio_set_level(LED_AUTO_STATUS_2, 1);
    
    // Animate D5-D9 LEDs progressively (1 second per frame)
    if (current_ms - last_anim_update_ms >= LED_ANIM_INTERVAL_MS) {
        last_anim_update_ms = current_ms;
        anim_led_index = (anim_led_index + 1) % 5;  // Cycle through 5 animation LEDs

        // Set animation LEDs based on current index
        // Index 0: D5 on
        // Index 1: D5+D6 on
        // Index 2: D5+D6+D7 on
        // Index 3: D5+D6+D7+D8 on
        // Index 4: D5+D6+D7+D8+D9 on
        gpio_set_level(LED_ANIM_D5, anim_led_index < 5 ? 1 : 0); // Always on in animation
        gpio_set_level(LED_ANIM_D6, anim_led_index >= 1 ? 1 : 0);
        gpio_set_level(LED_ANIM_D7, anim_led_index >= 2 ? 1 : 0);
        gpio_set_level(LED_ANIM_D8, anim_led_index >= 3 ? 1 : 0);
        gpio_set_level(LED_ANIM_D9, anim_led_index >= 4 ? 1 : 0);

        ESP_LOGD(TAG, "Auto mode animation frame: %d", anim_led_index);
    }
}

static void update_indicator_leds(uint32_t pump_percent)
{
    // Update LED indicators based on pump percentage
    if (pump_percent == 0) {
        // 0% = No LEDs on
        gpio_set_level(LED_INDICATOR_D2, 0);
        gpio_set_level(LED_INDICATOR_D3, 0);
        gpio_set_level(LED_INDICATOR_D4, 0);
        ESP_LOGI(TAG, "Indicator LEDs: OFF");
    }
    else if (pump_percent <= 30) {
        // 10%-30% = D2 on
        gpio_set_level(LED_INDICATOR_D2, 1);
        gpio_set_level(LED_INDICATOR_D3, 0);
        gpio_set_level(LED_INDICATOR_D4, 0);
        ESP_LOGI(TAG, "Indicator LEDs: D2 ON");
    }
    else if (pump_percent <= 60) {
        // 40%-60% = D2 and D3 on
        gpio_set_level(LED_INDICATOR_D2, 1);
        gpio_set_level(LED_INDICATOR_D3, 1);
        gpio_set_level(LED_INDICATOR_D4, 0);
        ESP_LOGI(TAG, "Indicator LEDs: D2 D3 ON");
    }
    else if (pump_percent < 100) {
        // 70%-90% = D2, D3, D4 all on
        gpio_set_level(LED_INDICATOR_D2, 1);
        gpio_set_level(LED_INDICATOR_D3, 1);
        gpio_set_level(LED_INDICATOR_D4, 1);
        ESP_LOGI(TAG, "Indicator LEDs: D2 D3 D4 ON");
    }
    else {
        // 100% = Setup for flashing (actual flashing done in main loop)
        ESP_LOGI(TAG, "Indicator LEDs: FLASHING MODE");
    }
}

static void update_flash_leds(uint32_t current_ms)
{
    // Flash all LEDs every 250ms when at 100%
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
    // In MANUAL mode: flash D3/D4 status LEDs as mode indicator
    // In AUTOMATIC mode: keep D3/D4 on (handled in update_animation_leds)
    if (control_mode == CONTROL_MANUAL) {
        if (current_ms - last_status_led_toggle_ms >= LED_FLASH_INTERVAL_MS) {
            status_led_state = !status_led_state;
            last_status_led_toggle_ms = current_ms;
            
            gpio_set_level(LED_AUTO_STATUS_1, status_led_state ? 1 : 0);
            gpio_set_level(LED_AUTO_STATUS_2, status_led_state ? 1 : 0);
        }
    }
}

static void init_pump(void)
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode = PWM_MODE,
        .timer_num = PWM_TIMER,
        .duty_resolution = PWM_RESOLUTION,
        .freq_hz = PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .speed_mode = PWM_MODE,
        .channel = PWM_CHANNEL,
        .timer_sel = PWM_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = PWM_LINE_GPIO,
        .duty = 0,
        .hpoint = 0,
        .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
    };
    ledc_channel_config(&ledc_channel);

    ESP_LOGI(TAG, "Pump PWM initialized at %d Hz", PWM_FREQ_HZ);
}

// Initialize PWM-synchronized INA reading system
static void init_ina_sync_reading(void)
{
    // Create binary semaphore for INA reading synchronization
    ina_read_semaphore = xSemaphoreCreateBinary();
    if (ina_read_semaphore == NULL) {
        ESP_LOGE(TAG, "Failed to create INA read semaphore");
        return;
    }
    ESP_LOGD(TAG, "INA read semaphore created");

    // Create high-resolution timer for PWM synchronization
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

    // Start timer with period matching PWM frequency (2ms at 500Hz)
    uint64_t timer_period_us = (PWM_PERIOD_MS * 1000);
    err = esp_timer_start_periodic(pwm_sync_timer, timer_period_us);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start PWM sync timer: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "PWM sync timer started: %llu us period (%.1f Hz)", timer_period_us, 1000000.0f / timer_period_us);

    // Create dedicated high-priority task for synchronized INA readings
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
    // Configure indicator LED outputs (D9, D8, D7)
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
    
    // Configure status LED outputs (D3, D4)
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
    
    // Configure animation LED outputs (D5-D9)
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
    
    // Configure button inputs with pull-up
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
    bool new_state = gpio_get_level(btn->pin) == 0;  // 0 = pressed, 1 = released
    
    // Detect falling edge: transition from released (true) to pressed (false)
    if (new_state == false && btn->stable_state == true) {
        btn->stable_state = false;  // Mark as pressed/locked
        return true;  // Button press detected
    }
    
    // Detect rising edge: transition from pressed (false) to released (true)
    if (new_state == true && btn->stable_state == false) {
        btn->stable_state = true;  // Mark as released/unlocked
    }
    
    return false;
}

void app_main(void)
{
    // Initialize I2C bus
    esp_err_t err = i2c_bus_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C bus: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "I2C bus initialized");

    // Scan I2C bus for devices
    ESP_LOGI(TAG, "Scanning I2C bus...");
    i2c_bus_scan(pdMS_TO_TICKS(50));

    // Initialize pump
    init_pump();

    // Initialize PWM-synchronized INA reading system
    init_ina_sync_reading();

    // Initialize buttons and LEDs
    init_buttons_and_led();

    // Initialize TMP117
    esp_err_t terr = tmp117_init(TMP117_ADDR);
    if (terr == ESP_OK) {
        ESP_LOGI(TAG, "TMP117 found and initialized at 0x%02X", TMP117_ADDR);
    } else {
        ESP_LOGE(TAG, "TMP117 not found at 0x%02X: %s", TMP117_ADDR, esp_err_to_name(terr));
    }

    // Initialize INA226 #1
    err = ina226_init(INA226_1_ADDR, INA226_PUMP_SHUNT_RESISTANCE_OHM);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "PUMP (INA1, 0x%02X) not found: %s", INA226_1_ADDR, esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "PUMP (INA1, 0x%02X) initialized", INA226_1_ADDR);
    }

    // Initialize INA226 #2
    err = ina226_init(INA226_2_ADDR, INA226_LOGIC_SHUNT_RESISTANCE_OHM);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "LOGIC (INA2, 0x%02X) not found: %s", INA226_2_ADDR, esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "LOGIC (INA2, 0x%02X) initialized", INA226_2_ADDR);
    }

    // Initialize BLE
    ble_init();
    ESP_LOGI(TAG, "BLE initialized");

    // Set initial pump speed
    set_pump_percent(PUMP_START_PERCENT);
    ESP_LOGI(TAG, "Pump initialized to %d%%", PUMP_START_PERCENT);

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
                    set_pump_percent(new_percent);
                    ESP_LOGI(TAG, "Pump increased to %d%%", new_percent);
                }

                if (button_pressed(&button_a3)) {
                    int32_t new_percent = (int32_t)current_pump_percent - PUMP_INCREMENT;
                    if (new_percent < 0) {
                        new_percent = 0;
                    }
                    set_pump_percent((uint32_t)new_percent);
                    ESP_LOGI(TAG, "Pump decreased to %d%%", (uint32_t)new_percent);
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
                    "Pump PWM: %u%% | TMP117: %.2f°C / %.2f°F\n"
                    "  PUMP : BUS=%.3fV  SHUNT=%.3fmV  CURRENT=%.3fmA  POWER=%.3fmW (smoothed)\n"
                    "  LOGIC: BUS=%.3fV  SHUNT=%.3fmV  CURRENT=%.3fmA  POWER=%.3fmW (smoothed)",
                    current_pump_percent,
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
                        current_pump_percent,
                        filtered_ina1_current_ma,
                        filtered_ina2_current_ma);
                    ble_notify(msg);
                }

            } else if (ina1_err == ESP_OK) {
                ESP_LOGI(
                    TAG,
                    "Pump PWM: %u%% | TMP117: %.2f°C / %.2f°F\n"
                    "  PUMP : BUS=%.3fV  SHUNT=%.3fmV  CURRENT=%.3fmA  POWER=%.3fmW (smoothed)",
                    current_pump_percent,
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
                    "Pump PWM: %u%% | TMP117: %.2f°C / %.2f°F\n"
                    "  LOGIC: BUS=%.3fV  SHUNT=%.3fmV  CURRENT=%.3fmA  POWER=%.3fmW (smoothed)",
                    current_pump_percent,
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
                ESP_LOGI(TAG, "Pump PWM: %u%% | TMP117: %.2f°C / %.2f°F",
                         current_pump_percent, temp_c, temp_f);
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
                    "Pump PWM: %u%% | TMP117: (read failed)\n"
                    "  PUMP : BUS=%.3fV  SHUNT=%.3fmV  CURRENT=%.3fmA  POWER=%.3fmW (smoothed)",
                    current_pump_percent,
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
                    "Pump PWM: %u%% | TMP117: (read failed)\n"
                    "  LOGIC: BUS=%.3fV  SHUNT=%.3fmV  CURRENT=%.3fmA  POWER=%.3fmW (smoothed)",
                    current_pump_percent,
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