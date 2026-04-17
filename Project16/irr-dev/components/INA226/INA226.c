#include <stdio.h>
#include "ina226.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

static const char *TAG = "INA226";

// INA226 I2C address (default: 0x40, can be configured with A0/A1 pins)
#define INA226_I2C_ADDR 0x40

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

// Calibration value (for Rshunt = 0.1 Ohm: calibration = 81.92 / (I_max * Rshunt))
// If max current = 20A with 0.1 Ohm shunt: calibration = 41
#define INA226_CALIBRATION_VALUE 41

static i2c_master_dev_handle_t ina226_handle = NULL;
static float current_threshold = 20.0f;  // Default: 20A
static float power_threshold = 200.0f;   // Default: 200W
static void (*low_power_callback)(float power) = NULL;
static void (*high_current_callback)(float current) = NULL;

/**
 * Write register to INA226
 */
static int ina226_write_register(uint8_t reg, uint16_t value)
{
    uint8_t data[3] = {reg, (uint8_t)(value >> 8), (uint8_t)(value & 0xFF)};
    esp_err_t ret = i2c_master_transmit(ina226_handle, data, 3, pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write register 0x%02x: %s", reg, esp_err_to_name(ret));
        return -1;
    }
    return 0;
}

/**
 * Read register from INA226
 */
static int ina226_read_register(uint8_t reg, uint16_t *value)
{
    uint8_t cmd = reg;
    uint8_t data[2] = {0};
    esp_err_t ret = i2c_master_transmit_receive(ina226_handle, &cmd, 1, data, 2, pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read register 0x%02x: %s", reg, esp_err_to_name(ret));
        return -1;
    }
    *value = (data[0] << 8) | data[1];
    return 0;
}

int ina226_init(void)
{
    ESP_LOGI(TAG, "Initializing INA226 power monitor");

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

    static i2c_master_bus_handle_t bus_handle = NULL;
    if (bus_handle == NULL) {
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));
    }

    // Add INA226 device to bus
    i2c_device_config_t ina226_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = INA226_I2C_ADDR,
        .scl_speed_hz = 100000,
    };

    esp_err_t ret = i2c_master_bus_add_device(bus_handle, &ina226_config, &ina226_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add INA226 to I2C bus: %s", esp_err_to_name(ret));
        return -1;
    }

    // Write configuration register
    if (ina226_write_register(INA226_CONFIG_REG, INA226_CONFIG_DEFAULT) != 0) {
        return -1;
    }

    // Write calibration register
    if (ina226_write_register(INA226_CALIBRATION_REG, INA226_CALIBRATION_VALUE) != 0) {
        return -1;
    }

    ESP_LOGI(TAG, "INA226 initialization complete");
    return 0;
}

float ina226_read_bus_voltage(void)
{
    uint16_t raw_value = 0;
    if (ina226_read_register(INA226_BUS_VOLT_REG, &raw_value) != 0) {
        return 0.0f;
    }
    // Bus voltage: 1.25mV per LSB
    float voltage = (raw_value >> 3) * 0.00125f;
    ESP_LOGI(TAG, "Bus voltage: %.3f V", voltage);
    return voltage;
}

float ina226_read_shunt_voltage(void)
{
    uint16_t raw_value = 0;
    if (ina226_read_register(INA226_SHUNT_VOLT_REG, &raw_value) != 0) {
        return 0.0f;
    }
    // Shunt voltage: 2.5µV per LSB (convert to mV = 0.0025 mV per LSB)
    float voltage = (int16_t)raw_value * 0.0025f;
    ESP_LOGI(TAG, "Shunt voltage: %.3f mV", voltage);
    return voltage;
}

float ina226_read_current(void)
{
    uint16_t raw_value = 0;
    if (ina226_read_register(INA226_CURRENT_REG, &raw_value) != 0) {
        return 0.0f;
    }
    // Current in A (depends on calibration register)
    // Default calibration: 1 LSB = 1mA
    float current = (int16_t)raw_value * 0.001f;
    ESP_LOGI(TAG, "Current: %.3f A", current);

    // Check for high current alert
    if (current > current_threshold && high_current_callback) {
        ESP_LOGW(TAG, "HIGH CURRENT ALERT: %.3f A exceeds threshold %.3f A", current, current_threshold);
        high_current_callback(current);
    }

    return current;
}

float ina226_read_power(void)
{
    uint16_t raw_value = 0;
    if (ina226_read_register(INA226_POWER_REG, &raw_value) != 0) {
        return 0.0f;
    }
    // Power: 25mW per LSB
    float power = raw_value * 0.025f;
    ESP_LOGI(TAG, "Power: %.3f W", power);

    // Check for low power alert
    if (power < power_threshold && low_power_callback) {
        ESP_LOGW(TAG, "LOW POWER ALERT: %.3f W below threshold %.3f W", power, power_threshold);
        low_power_callback(power);
    }

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
