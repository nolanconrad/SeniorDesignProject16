#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include "ina226.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

static const char *TAG = "INA226";

// Two INA226 devices on the same bus with unique I2C addresses.
#define INA226_DEVICE_COUNT 2
#define INA226_DEVICE_PRIMARY 0
#define INA226_DEVICE_SECONDARY 1

// INA226 Register addresses
#define INA226_CONFIG_REG       0x00
#define INA226_SHUNT_VOLT_REG   0x01
#define INA226_BUS_VOLT_REG     0x02
#define INA226_POWER_REG        0x03
#define INA226_CURRENT_REG      0x04
#define INA226_CALIBRATION_REG  0x05
#define INA226_MASK_ENABLE_REG  0x06
#define INA226_ALERT_LIMIT_REG  0x07

// Configuration Register defaults
#define INA226_CONFIG_DEFAULT   0x4127  // Average 64 samples, 1.1ms conversion time

// Calibration for high sensitivity with Rshunt = 0.01 Ohm.
// Target: 0.1 mA/LSB => Current_LSB = 0.0001 A
// Calibration = 0.00512 / (Current_LSB * Rshunt) = 5120
#define INA226_CALIBRATION_VALUE 5120

// Derived from the configured calibration and shunt resistor.
// Current_LSB = 0.1 mA/LSB, Power_LSB = 25 * Current_LSB
#define INA226_CURRENT_LSB_A 0.0001f
#define INA226_POWER_LSB_W   0.0025f

#define INA226_ZERO_OFFSET_SAMPLES 32
#define INA226_ZERO_CLAMP_A 0.002f
#define INA226_OFFSET_ACTIVE_CURRENT_A 0.05f

static const uint8_t ina226_addresses[INA226_DEVICE_COUNT] = {0x40, 0x41};
static const char *ina226_device_labels[INA226_DEVICE_COUNT] = {"PUMP", "LOGIC"};
static i2c_master_dev_handle_t ina226_handles[INA226_DEVICE_COUNT] = {NULL};
static i2c_master_bus_handle_t ina226_bus_handle = NULL;
static bool ina226_initialized[INA226_DEVICE_COUNT] = {false};
static float current_threshold = 3.6f;   // Default: 3.6A
static float power_threshold = 200.0f;   // Default: 200W
static float current_offset_a[INA226_DEVICE_COUNT] = {0.0f};
static void (*low_power_callback)(float power) = NULL;
static void (*high_current_callback)(float current) = NULL;

static int ina226_read_register(uint8_t device_index, uint8_t reg, uint16_t *value);

static bool ina226_is_valid_index(uint8_t device_index)
{
    return device_index < INA226_DEVICE_COUNT;
}

static const char *ina226_device_label(uint8_t device_index)
{
    if (!ina226_is_valid_index(device_index)) {
        return "UNKNOWN";
    }
    return ina226_device_labels[device_index];
}

static float ina226_raw_current_amps(uint8_t device_index)
{
    uint16_t raw_value = 0;
    if (ina226_read_register(device_index, INA226_CURRENT_REG, &raw_value) != 0) {
        return 0.0f;
    }
    return (int16_t)raw_value * INA226_CURRENT_LSB_A;
}

static void ina226_calibrate_zero_offset(uint8_t device_index)
{
    float sum_current = 0.0f;
    int valid_samples = 0;

    for (int i = 0; i < INA226_ZERO_OFFSET_SAMPLES; i++) {
        uint16_t raw_value = 0;
        if (ina226_read_register(device_index, INA226_CURRENT_REG, &raw_value) == 0) {
            sum_current += (int16_t)raw_value * INA226_CURRENT_LSB_A;
            valid_samples++;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    if (valid_samples > 0) {
        current_offset_a[device_index] = sum_current / (float)valid_samples;
        if (fabsf(current_offset_a[device_index]) > INA226_OFFSET_ACTIVE_CURRENT_A) {
            ESP_LOGW(TAG, "Large zero-current offset on %s (0x%02X): %.4f A. Ensure load is OFF during calibration.",
                     ina226_device_label(device_index), ina226_addresses[device_index], current_offset_a[device_index]);
        }
        ESP_LOGI(TAG, "Current zero offset calibrated for %s (0x%02X): %.4f A (%d samples)",
                 ina226_device_label(device_index), ina226_addresses[device_index], current_offset_a[device_index], valid_samples);
    } else {
        current_offset_a[device_index] = 0.0f;
        ESP_LOGW(TAG, "Current zero offset calibration failed for %s (0x%02X), using 0.0 A",
                 ina226_device_label(device_index), ina226_addresses[device_index]);
    }
}

/**
 * Write register to INA226
 */
static int ina226_write_register(uint8_t device_index, uint8_t reg, uint16_t value)
{
    uint8_t data[3] = {reg, (uint8_t)(value >> 8), (uint8_t)(value & 0xFF)};
    esp_err_t ret = i2c_master_transmit(ina226_handles[device_index], data, 3, pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write addr 0x%02X reg 0x%02X: %s", ina226_addresses[device_index], reg, esp_err_to_name(ret));
        return -1;
    }
    return 0;
}

/**
 * Read register from INA226
 */
static int ina226_read_register(uint8_t device_index, uint8_t reg, uint16_t *value)
{
    uint8_t cmd = reg;
    uint8_t data[2] = {0};
    esp_err_t ret = i2c_master_transmit_receive(ina226_handles[device_index], &cmd, 1, data, 2, pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read addr 0x%02X reg 0x%02X: %s", ina226_addresses[device_index], reg, esp_err_to_name(ret));
        return -1;
    }
    *value = (data[0] << 8) | data[1];
    return 0;
}

static int ina226_init_device(uint8_t device_index)
{
    if (!ina226_is_valid_index(device_index)) {
        ESP_LOGE(TAG, "Invalid INA226 device index: %u", device_index);
        return -1;
    }

    if (ina226_initialized[device_index] && ina226_handles[device_index] != NULL) {
        ESP_LOGW(TAG, "INA226 %s (0x%02X) already initialized", ina226_device_label(device_index), ina226_addresses[device_index]);
        return 0;
    }

    i2c_device_config_t ina226_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ina226_addresses[device_index],
        .scl_speed_hz = 100000,
    };

    esp_err_t ret = i2c_master_bus_add_device(ina226_bus_handle, &ina226_config, &ina226_handles[device_index]);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add INA226 %s (0x%02X): %s", ina226_device_label(device_index), ina226_addresses[device_index], esp_err_to_name(ret));
        return -1;
    }

    if (ina226_write_register(device_index, INA226_CONFIG_REG, INA226_CONFIG_DEFAULT) != 0) {
        return -1;
    }

    if (ina226_write_register(device_index, INA226_CALIBRATION_REG, INA226_CALIBRATION_VALUE) != 0) {
        return -1;
    }

    uint16_t config_verify = 0;
    uint16_t calibration_verify = 0;
    if (ina226_read_register(device_index, INA226_CONFIG_REG, &config_verify) != 0 ||
        ina226_read_register(device_index, INA226_CALIBRATION_REG, &calibration_verify) != 0) {
        ESP_LOGE(TAG, "Failed to verify INA226 %s (0x%02X) register configuration",
                 ina226_device_label(device_index), ina226_addresses[device_index]);
        return -1;
    }
    if (calibration_verify != INA226_CALIBRATION_VALUE) {
        ESP_LOGE(TAG, "Calibration mismatch on %s (0x%02X). Wrote: %u Read: %u",
                 ina226_device_label(device_index), ina226_addresses[device_index], INA226_CALIBRATION_VALUE, calibration_verify);
        return -1;
    }

    vTaskDelay(pdMS_TO_TICKS(100));
    ina226_calibrate_zero_offset(device_index);
    ina226_initialized[device_index] = true;

    ESP_LOGI(TAG, "INA226 %s initialized at 0x%02X. Config=0x%04X Cal=0x%04X",
             ina226_device_label(device_index), ina226_addresses[device_index], config_verify, calibration_verify);
    return 0;
}

int ina226_init(void)
{
    ESP_LOGI(TAG, "Initializing INA226 power monitors (0x40, 0x41)");

    // Configure I2C bus if not already done
    i2c_master_bus_config_t i2c_bus_config = {
        .i2c_port = I2C_NUM_0,
        .scl_io_num = 6,
        .sda_io_num = 5,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .flags.enable_internal_pullup = true,
    };

    if (ina226_bus_handle == NULL) {
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &ina226_bus_handle));
    }

    for (uint8_t i = 0; i < INA226_DEVICE_COUNT; i++) {
        if (ina226_init_device(i) != 0) {
            return -1;
        }
    }

    ESP_LOGI(TAG, "INA226 dual-device initialization complete");
    return 0;
}

int ina226_recalibrate_zero_offset(void)
{
    if (!ina226_initialized[INA226_DEVICE_PRIMARY] || !ina226_initialized[INA226_DEVICE_SECONDARY]) {
        ESP_LOGE(TAG, "Cannot calibrate offsets before initialization");
        return -1;
    }

    ina226_calibrate_zero_offset(INA226_DEVICE_PRIMARY);
    ina226_calibrate_zero_offset(INA226_DEVICE_SECONDARY);
    return 0;
}

int ina226_recalibrate_zero_offset_device(uint8_t device_index)
{
    if (!ina226_is_valid_index(device_index) || !ina226_initialized[device_index]) {
        ESP_LOGE(TAG, "Cannot calibrate offset for invalid/uninitialized device %u", device_index);
        return -1;
    }

    ina226_calibrate_zero_offset(device_index);
    return 0;
}

float ina226_read_bus_voltage(void)
{
    return ina226_read_bus_voltage_device(INA226_DEVICE_PRIMARY);
}

float ina226_read_shunt_voltage(void)
{
    return ina226_read_shunt_voltage_device(INA226_DEVICE_PRIMARY);
}

float ina226_read_current(void)
{
    float current = ina226_read_current_device(INA226_DEVICE_PRIMARY);

    // Check for high current alert
    if (fabsf(current) > current_threshold && high_current_callback) {
        ESP_LOGW(TAG, "HIGH CURRENT ALERT: %.3f A exceeds magnitude threshold %.3f A", current, current_threshold);
        high_current_callback(current);
    }

    return current;
}

float ina226_read_power(void)
{
    float power = ina226_read_power_device(INA226_DEVICE_PRIMARY);

    // Check for low power alert
    if (power < power_threshold && low_power_callback) {
        ESP_LOGW(TAG, "LOW POWER ALERT: %.3f W below threshold %.3f W", power, power_threshold);
        low_power_callback(power);
    }

    return power;
}

float ina226_read_bus_voltage_device(uint8_t device_index)
{
    if (!ina226_is_valid_index(device_index) || !ina226_initialized[device_index]) {
        ESP_LOGE(TAG, "INA226 bus voltage read requested for invalid/uninitialized device %u", device_index);
        return 0.0f;
    }

    uint16_t raw_value = 0;
    if (ina226_read_register(device_index, INA226_BUS_VOLT_REG, &raw_value) != 0) {
        return 0.0f;
    }

    // Bus voltage: 1.25mV per LSB
    float voltage = (raw_value >> 3) * 0.00125f;
    ESP_LOGI(TAG, "Bus voltage [%s 0x%02X]: %.3f V", ina226_device_label(device_index), ina226_addresses[device_index], voltage);
    return voltage;
}

float ina226_read_shunt_voltage_device(uint8_t device_index)
{
    if (!ina226_is_valid_index(device_index) || !ina226_initialized[device_index]) {
        ESP_LOGE(TAG, "INA226 shunt voltage read requested for invalid/uninitialized device %u", device_index);
        return 0.0f;
    }

    uint16_t raw_value = 0;
    if (ina226_read_register(device_index, INA226_SHUNT_VOLT_REG, &raw_value) != 0) {
        return 0.0f;
    }

    // Shunt voltage: 2.5uV per LSB (convert to mV = 0.0025 mV per LSB)
    float voltage = (int16_t)raw_value * 0.0025f;
    ESP_LOGI(TAG, "Shunt voltage [%s 0x%02X]: %.3f mV", ina226_device_label(device_index), ina226_addresses[device_index], voltage);
    return voltage;
}

float ina226_read_current_device(uint8_t device_index)
{
    if (!ina226_is_valid_index(device_index) || !ina226_initialized[device_index]) {
        ESP_LOGE(TAG, "INA226 current read requested for invalid/uninitialized device %u", device_index);
        return 0.0f;
    }

    // Current in A based on the programmed calibration register and per-device zero-offset compensation.
    float current = ina226_raw_current_amps(device_index) - current_offset_a[device_index];
    if (fabsf(current) < INA226_ZERO_CLAMP_A) {
        current = 0.0f;
    }
    ESP_LOGI(TAG, "Current [%s 0x%02X]: %.3f A", ina226_device_label(device_index), ina226_addresses[device_index], current);
    return current;
}

float ina226_read_power_device(uint8_t device_index)
{
    if (!ina226_is_valid_index(device_index) || !ina226_initialized[device_index]) {
        ESP_LOGE(TAG, "INA226 power read requested for invalid/uninitialized device %u", device_index);
        return 0.0f;
    }

    uint16_t raw_value = 0;
    if (ina226_read_register(device_index, INA226_POWER_REG, &raw_value) != 0) {
        return 0.0f;
    }

    // Power in W based on the configured current LSB.
    float power = raw_value * INA226_POWER_LSB_W;
    ESP_LOGI(TAG, "Power [%s 0x%02X]: %.3f W", ina226_device_label(device_index), ina226_addresses[device_index], power);
    return power;
}

void ina226_register_low_power_callback(void (*callback)(float power))
{
    low_power_callback = callback;
    ESP_LOGI(TAG, "Low power callback registered");
}

void ina226_register_high_current_callback(void (*callback)(float current))
{
    high_current_callback = callback;
    ESP_LOGI(TAG, "High current callback registered");
}

void ina226_set_current_threshold(float threshold)
{
    current_threshold = threshold;
    ESP_LOGI(TAG, "Current threshold set to %.3f A", threshold);
}

void ina226_set_power_threshold(float threshold)
{
    power_threshold = threshold;
    ESP_LOGI(TAG, "Power threshold set to %.3f W", threshold);
}
