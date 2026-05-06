#include <stdio.h>
#include <stdbool.h>

#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "tmp117.h"
#include "ina226.h"
#include "i2c_bus.h"
#include "pumpOperation.h"
#include "ble_handler.h"

static const char *TAG = "SYSTEM";

#define BUTTON_D2 GPIO_NUM_5
#define BUTTON_A2 GPIO_NUM_3
#define BUTTON_A3 GPIO_NUM_4

#define LED_INDICATOR_D2 GPIO_NUM_18
#define LED_INDICATOR_D3 GPIO_NUM_17
#define LED_INDICATOR_D4 GPIO_NUM_10

#define LED_AUTO_STATUS_1 GPIO_NUM_6
#define LED_AUTO_STATUS_2 GPIO_NUM_7

#define LED_ANIM_D5 GPIO_NUM_8
#define LED_ANIM_D6 GPIO_NUM_9
#define LED_ANIM_D7 GPIO_NUM_10
#define LED_ANIM_D8 GPIO_NUM_17
#define LED_ANIM_D9 GPIO_NUM_18

#define PUMP_INCREMENT 25
#define PUMP_START_PERCENT 0

#define TMP117_ADDR_PRIMARY   0x48
#define TMP117_ADDR_SECONDARY 0x49

#define INA226_PUMP_ADDR    0x40
#define INA226_BATTERY_ADDR 0x41
#define INA226_SHUNT_RESISTANCE_OHM 1.3f

#define PUMP_OVERCURRENT_LIMIT_A 0.75f
#define BATTERY_LOW_VOLTAGE_LIMIT_V 10.5f
#define INA226_SHUNT_LIMIT_MV 81.92f

#define BUTTON_POLL_INTERVAL_MS 50
#define SENSOR_READ_INTERVAL_MS 2000
#define STARTUP_RETRY_DELAY_MS 1000
#define BLE_TEMP_UPDATE_INTERVAL_MS 3000

typedef enum {
    CONTROL_MANUAL = 0,
    CONTROL_AUTOMATIC = 1
} control_mode_t;

#define TEMP_STEP_98F 98.0f
#define TEMP_STEP_99F 99.0f
#define TEMP_STEP_100F 100.0f
#define TEMP_STEP_101F 101.0f

#define PUMP_25_PERCENT 25
#define PUMP_50_PERCENT 50
#define PUMP_75_PERCENT 75
#define PUMP_100_PERCENT 100

#define LED_FLASH_INTERVAL_MS 250

typedef struct {
    gpio_num_t pin;
    bool stable_state;
    bool current_state;
} button_state_t;

static uint8_t s_tmp117_addr1 = TMP117_ADDR_PRIMARY;
static uint8_t s_tmp117_addr2 = TMP117_ADDR_SECONDARY;
static bool s_tmp117_found1 = false;
static bool s_tmp117_found2 = false;

static bool s_ina226_pump_found = false;
static bool s_ina226_battery_found = false;

static bool battery_filter_initialized = false;
static float filtered_battery_voltage_v = 0.0f;
static float latest_battery_percent = 0.0f;

static bool pump_fault_active = false;
static bool pump_register_overflow_fault = false;
static bool pump_shunt_voltage_fault = false;
static bool battery_low_voltage_fault = false;
static float latest_pump_current_a = 0.0f;

static uint32_t current_pump_percent = PUMP_START_PERCENT;
static control_mode_t control_mode = CONTROL_MANUAL;
static uint32_t last_flash_toggle_ms = 0;
static bool flash_state = false;

static button_state_t button_d2 = {BUTTON_D2, true, true};
static button_state_t button_a2 = {BUTTON_A2, true, true};
static button_state_t button_a3 = {BUTTON_A3, true, true};

#define BATTERY_COUNTDOWN_SECONDS 180
static uint32_t battery_timer_start_ms = 0;
static bool battery_timer_active = true;
static uint32_t last_battery_blink_ms = 0;
static bool battery_blink_state = false;

float g_latest_temp_f = 0.0f;

static void update_animation_leds(uint32_t current_ms);
static void update_pump_level_leds(uint32_t pump_percent);
static void update_mode_led(uint32_t current_ms);
static void refresh_pump_output_state(void);

static void stop_pump_for_fault(void)
{
    current_pump_percent = 0;
    pump_operation_stop();
    update_pump_level_leds(0);
}

static float battery_percent_from_voltage(float voltage_v)
{
    float percent = ((voltage_v - 9.9f) / (12.6f - 9.9f)) * 100.0f;

    if (percent > 100.0f) percent = 100.0f;
    if (percent < 0.0f) percent = 0.0f;

    return percent;
}

static void update_battery_from_ina(float bus_voltage_v, float current_a)
{
    if (bus_voltage_v < 9.0f || bus_voltage_v > 13.5f) {
        printf("Battery INA ignored bad voltage: %.2fV\n", bus_voltage_v);
        return;
    }

    if (!battery_filter_initialized ||
        filtered_battery_voltage_v < 9.0f ||
        filtered_battery_voltage_v > 13.5f) {

        filtered_battery_voltage_v = bus_voltage_v;
        battery_filter_initialized = true;
    } else {
        float alpha = 0.05f;
        if (current_a > 0.20f) {
            alpha = 0.01f;
        }

        filtered_battery_voltage_v =
            alpha * bus_voltage_v + (1.0f - alpha) * filtered_battery_voltage_v;
    }

    latest_battery_percent = battery_percent_from_voltage(filtered_battery_voltage_v);
}

static void toggle_led(void)
{
    control_mode = (control_mode == CONTROL_MANUAL) ? CONTROL_AUTOMATIC : CONTROL_MANUAL;
    refresh_pump_output_state();

    if (control_mode == CONTROL_MANUAL) {
        update_pump_level_leds(current_pump_percent);
    }
}

static void update_pump_auto(float temp_f)
{
    if (pump_fault_active) {
        return;
    }

    uint32_t new_pump_percent = current_pump_percent;

    if (temp_f < TEMP_STEP_98F) {
        new_pump_percent = 0;
    } else if (temp_f < TEMP_STEP_99F) {
        new_pump_percent = PUMP_25_PERCENT;
    } else if (temp_f < TEMP_STEP_100F) {
        new_pump_percent = PUMP_50_PERCENT;
    } else if (temp_f < TEMP_STEP_101F) {
        new_pump_percent = PUMP_75_PERCENT;
    } else {
        new_pump_percent = PUMP_100_PERCENT;
    }

    if (new_pump_percent != current_pump_percent) {
        current_pump_percent = new_pump_percent;
        refresh_pump_output_state();
    }
}

static void update_animation_leds(uint32_t current_ms)
{
    if (battery_timer_active) {
        uint32_t elapsed_ms = current_ms - battery_timer_start_ms;
        uint32_t elapsed_s = elapsed_ms / 1000;
        uint32_t remaining_s = (elapsed_s < BATTERY_COUNTDOWN_SECONDS)
            ? (BATTERY_COUNTDOWN_SECONDS - elapsed_s)
            : 0;

        if (remaining_s > 120) {
            gpio_set_level(LED_AUTO_STATUS_1, 1);
            gpio_set_level(LED_AUTO_STATUS_2, 1);
            gpio_set_level(LED_ANIM_D5, 1);
        } else if (remaining_s > 60) {
            gpio_set_level(LED_AUTO_STATUS_1, 1);
            gpio_set_level(LED_AUTO_STATUS_2, 1);
            gpio_set_level(LED_ANIM_D5, 0);
        } else if (remaining_s > 0) {
            gpio_set_level(LED_AUTO_STATUS_2, 0);
            gpio_set_level(LED_ANIM_D5, 0);

            if (current_ms - last_battery_blink_ms >= LED_FLASH_INTERVAL_MS) {
                battery_blink_state = !battery_blink_state;
                last_battery_blink_ms = current_ms;
            }

            gpio_set_level(LED_AUTO_STATUS_1, battery_blink_state ? 1 : 0);
        } else {
            battery_timer_active = false;
            gpio_set_level(LED_AUTO_STATUS_1, 0);
            gpio_set_level(LED_AUTO_STATUS_2, 0);
            gpio_set_level(LED_ANIM_D5, 0);
        }
    }
}

static void update_pump_level_leds(uint32_t pump_percent)
{
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
    if (control_mode == CONTROL_MANUAL) {
        if (current_ms - last_flash_toggle_ms >= LED_FLASH_INTERVAL_MS) {
            flash_state = !flash_state;
            last_flash_toggle_ms = current_ms;
        }

        gpio_set_level(LED_ANIM_D9, flash_state ? 1 : 0);
    } else {
        gpio_set_level(LED_ANIM_D9, 1);
    }
}

static void init_buttons_and_led(void)
{
    gpio_config_t indicator_config = {
        .pin_bit_mask = (1ULL << LED_INDICATOR_D2) |
                        (1ULL << LED_INDICATOR_D3) |
                        (1ULL << LED_INDICATOR_D4),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&indicator_config);

    gpio_config_t status_led_config = {
        .pin_bit_mask = (1ULL << LED_AUTO_STATUS_1) |
                        (1ULL << LED_AUTO_STATUS_2),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&status_led_config);

    gpio_config_t anim_led_config = {
        .pin_bit_mask = (1ULL << LED_ANIM_D5) |
                        (1ULL << LED_ANIM_D6) |
                        (1ULL << LED_ANIM_D7) |
                        (1ULL << LED_ANIM_D8) |
                        (1ULL << LED_ANIM_D9),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&anim_led_config);

    gpio_set_level(LED_INDICATOR_D2, 0);
    gpio_set_level(LED_INDICATOR_D3, 0);
    gpio_set_level(LED_INDICATOR_D4, 0);
    gpio_set_level(LED_AUTO_STATUS_1, 0);
    gpio_set_level(LED_AUTO_STATUS_2, 0);
    gpio_set_level(LED_ANIM_D5, 0);
    gpio_set_level(LED_ANIM_D6, 0);
    gpio_set_level(LED_ANIM_D7, 0);
    gpio_set_level(LED_ANIM_D8, 0);
    gpio_set_level(LED_ANIM_D9, 0);

    gpio_config_t button_config = {
        .pin_bit_mask = (1ULL << BUTTON_D2) |
                        (1ULL << BUTTON_A2) |
                        (1ULL << BUTTON_A3),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&button_config);
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
    if (pump_fault_active && current_pump_percent > 0) {
        current_pump_percent = 0;
    }

    if (pump_operation_set_duty_percent((uint8_t)current_pump_percent) != ESP_OK) {
        return;
    }

    if (control_mode == CONTROL_MANUAL) {
        update_pump_level_leds(current_pump_percent);
    }
}

void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_NONE);

    esp_err_t err = ESP_OK;

    while ((err = i2c_bus_init()) != ESP_OK) {
        printf("Failed to initialize I2C bus: %s. Retrying...\n", esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(STARTUP_RETRY_DELAY_MS));
    }

    while ((err = pump_operation_init()) != ESP_OK) {
        printf("Failed to initialize pump output: %s. Retrying...\n", esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(STARTUP_RETRY_DELAY_MS));
    }

    init_buttons_and_led();

    battery_timer_start_ms = esp_timer_get_time() / 1000;

    if (tmp117_init(s_tmp117_addr1) == ESP_OK) {
        s_tmp117_found1 = true;
    }

    if (tmp117_init(s_tmp117_addr2) == ESP_OK) {
        s_tmp117_found2 = true;
    }

    if (ina226_init(INA226_PUMP_ADDR, INA226_SHUNT_RESISTANCE_OHM) == ESP_OK) {
        s_ina226_pump_found = true;
        printf("INA226 pump monitor found at 0x%02X\n", INA226_PUMP_ADDR);
    } else {
        printf("INA226 pump monitor not found at 0x%02X\n", INA226_PUMP_ADDR);
    }

    if (ina226_init(INA226_BATTERY_ADDR, INA226_SHUNT_RESISTANCE_OHM) == ESP_OK) {
        s_ina226_battery_found = true;
        printf("INA226 battery monitor found at 0x%02X\n", INA226_BATTERY_ADDR);
    } else {
        printf("INA226 battery monitor not found at 0x%02X\n", INA226_BATTERY_ADDR);
    }

    ble_init();

    while ((err = pump_operation_set_duty_percent((uint8_t)PUMP_START_PERCENT)) != ESP_OK) {
        printf("Failed to set initial pump speed: %s. Retrying...\n", esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(STARTUP_RETRY_DELAY_MS));
    }

    current_pump_percent = PUMP_START_PERCENT;
    update_pump_level_leds(current_pump_percent);

    uint32_t last_ble_update_ms = 0;

    while (1) {
        const int poll_loops = SENSOR_READ_INTERVAL_MS / BUTTON_POLL_INTERVAL_MS;

        for (int i = 0; i < poll_loops; i++) {
            uint32_t now_ms = esp_log_timestamp();

            update_mode_led(now_ms);

            if (control_mode == CONTROL_MANUAL) {
                update_pump_level_leds(current_pump_percent);
            } else {
                update_animation_leds(now_ms);
            }

            if (button_pressed(&button_d2)) {
                toggle_led();
            }

            if (control_mode == CONTROL_MANUAL) {
                if (!pump_fault_active && button_pressed(&button_a2)) {
                    uint32_t new_percent = current_pump_percent + PUMP_INCREMENT;
                    if (new_percent > 100) {
                        new_percent = 100;
                    }

                    current_pump_percent = new_percent;
                    refresh_pump_output_state();
                }

                if (button_pressed(&button_a3)) {
                    int32_t new_percent = (int32_t)current_pump_percent - PUMP_INCREMENT;
                    if (new_percent < 0) {
                        new_percent = 0;
                    }

                    current_pump_percent = (uint32_t)new_percent;
                    refresh_pump_output_state();
                }
            }

            vTaskDelay(pdMS_TO_TICKS(BUTTON_POLL_INTERVAL_MS));
        }

        float temp_c1 = 0.0f;
        float temp_c2 = 0.0f;
        bool valid1 = false;
        bool valid2 = false;

        if (s_tmp117_found1 && tmp117_read_temperature_c(s_tmp117_addr1, &temp_c1) == ESP_OK) {
            valid1 = true;
        }

        if (s_tmp117_found2 && tmp117_read_temperature_c(s_tmp117_addr2, &temp_c2) == ESP_OK) {
            valid2 = true;
        }

        if (valid1 || valid2) {
            float avg_temp_c = valid1 && valid2
                ? (temp_c1 + temp_c2) / 2.0f
                : valid1 ? temp_c1 : temp_c2;

            float avg_temp_f = (avg_temp_c * 9.0f / 5.0f) + 32.0f;
            g_latest_temp_f = avg_temp_f;

            printf("TMP1(0x%02X): %s | TMP2(0x%02X): %s | AVG: %.2fC / %.2fF | Pump: %lu%%\n",
                   s_tmp117_addr1,
                   valid1 ? "OK" : "N/A",
                   s_tmp117_addr2,
                   valid2 ? "OK" : "N/A",
                   avg_temp_c,
                   avg_temp_f,
                   (unsigned long)current_pump_percent);

            if (control_mode == CONTROL_AUTOMATIC) {
                update_pump_auto(avg_temp_f);
            }
        } else {
            printf("TMP117 sensors: N/A | Pump: %lu%%\n",
                   (unsigned long)current_pump_percent);
        }

        if (s_ina226_pump_found) {
            ina226_measurement_t pump_meas;

            esp_err_t pump_err = ina226_read_measurement(
                INA226_PUMP_ADDR,
                INA226_SHUNT_RESISTANCE_OHM,
                &pump_meas
            );

            if (pump_err == ESP_OK) {
                pump_register_overflow_fault = false;

                latest_pump_current_a = pump_meas.current_a;
                if (latest_pump_current_a < 0.0f) {
                    latest_pump_current_a = -latest_pump_current_a;
                }

                float pump_shunt_mV = pump_meas.shunt_voltage_v * 1000.0f;
                if (pump_shunt_mV < 0.0f) {
                    pump_shunt_mV = -pump_shunt_mV;
                }

                pump_shunt_voltage_fault = pump_shunt_mV >= INA226_SHUNT_LIMIT_MV;

                bool pump_current_fault =
                    latest_pump_current_a >= PUMP_OVERCURRENT_LIMIT_A;

                pump_fault_active =
                    pump_current_fault ||
                    pump_register_overflow_fault ||
                    pump_shunt_voltage_fault ||
                    battery_low_voltage_fault;

                float pump_current_mA = latest_pump_current_a * 1000.0f;
                float pump_power_mW = pump_meas.power_w * 1000.0f;

                printf("Pump INA (0x40): %.2fV | Shunt=%.2fmV | %.1fmA | %.2fmW | Fault=%d | OverCurrent=%d | ShuntFault=%d | RegOverflow=%d\n",
                       pump_meas.bus_voltage_v,
                       pump_shunt_mV,
                       pump_current_mA,
                       pump_power_mW,
                       pump_fault_active ? 1 : 0,
                       pump_current_fault ? 1 : 0,
                       pump_shunt_voltage_fault ? 1 : 0,
                       pump_register_overflow_fault ? 1 : 0);

                if (pump_fault_active) {
                    stop_pump_for_fault();
                    printf("PUMP FAULT: current=%.3fA shunt=%.2fmV lowV=%d. Pump stopped.\n",
                           latest_pump_current_a,
                           pump_shunt_mV,
                           battery_low_voltage_fault ? 1 : 0);
                }
            } else if (pump_err == ESP_ERR_INVALID_RESPONSE) {
                pump_register_overflow_fault = true;
                pump_fault_active = true;
                stop_pump_for_fault();

                printf("PUMP FAULT: INA226 shunt register overflow/saturation. Pump stopped.\n");
            } else {
                printf("Pump INA (0x40): read failed: %s\n", esp_err_to_name(pump_err));
            }
        }

        if (s_ina226_battery_found) {
            ina226_measurement_t batt_meas;

            if (ina226_read_measurement(
                    INA226_BATTERY_ADDR,
                    INA226_SHUNT_RESISTANCE_OHM,
                    &batt_meas) == ESP_OK) {

                update_battery_from_ina(
                    batt_meas.bus_voltage_v,
                    batt_meas.current_a
                );

                battery_low_voltage_fault =
                    batt_meas.bus_voltage_v <= BATTERY_LOW_VOLTAGE_LIMIT_V;

                if (battery_low_voltage_fault) {
                    pump_fault_active = true;
                    stop_pump_for_fault();

                    printf("BATTERY FAULT: voltage %.2fV <= %.2fV. Pump stopped.\n",
                           batt_meas.bus_voltage_v,
                           BATTERY_LOW_VOLTAGE_LIMIT_V);
                }

                float current_mA = batt_meas.current_a * 1000.0f;
                float power_mW = batt_meas.power_w * 1000.0f;
                float voltage_drop_mV =
                    (batt_meas.bus_voltage_v - filtered_battery_voltage_v) * 1000.0f;

                printf("Battery INA (0x41): %.2fV raw | %.2fV filtered | dV=%.2fmV | %.0f%% | %.1fmA | %.2fmW | LowV=%d\n",
                       batt_meas.bus_voltage_v,
                       filtered_battery_voltage_v,
                       voltage_drop_mV,
                       latest_battery_percent,
                       current_mA,
                       power_mW,
                       battery_low_voltage_fault ? 1 : 0);
            } else {
                printf("Battery INA (0x41): read failed\n");
            }
        }

        uint32_t now_ms = esp_log_timestamp();
        if (ble_is_connected() &&
            (now_ms - last_ble_update_ms >= BLE_TEMP_UPDATE_INTERVAL_MS)) {

            last_ble_update_ms = now_ms;

            char msg[128];
            snprintf(
                msg,
                sizeof(msg),
                "TempF=%.1f,Batt=%.0f,Curr=%.2f,Fault=%d,LowV=%d,RegOverflow=%d,ShuntFault=%d",
                g_latest_temp_f,
                latest_battery_percent,
                latest_pump_current_a,
                pump_fault_active ? 1 : 0,
                battery_low_voltage_fault ? 1 : 0,
                pump_register_overflow_fault ? 1 : 0,
                pump_shunt_voltage_fault ? 1 : 0
            );

            ble_notify(msg);
        }
    }
}