#include "ina226.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "i2c_bus.h"

#define INA226_REG_CONFIG 0x00
#define INA226_REG_SHUNT_VOLTAGE 0x01
#define INA226_REG_BUS_VOLTAGE 0x02
#define INA226_REG_POWER 0x03
#define INA226_REG_CURRENT 0x04
#define INA226_REG_CALIBRATION 0x05
#define INA226_REG_MANUFACTURER_ID 0xFE
#define INA226_REG_DIE_ID 0xFF

#define INA226_EXPECTED_MANUFACTURER_ID 0x5449
#define INA226_EXPECTED_DIE_ID 0x2260

#define INA226_CONFIG_DEFAULT 0x4127
#define INA226_SHUNT_VOLTAGE_LSB_V 0.0000025f
#define INA226_BUS_VOLTAGE_LSB_V 0.00125f
#define INA226_CURRENT_LSB_A 0.00000153f
#define INA226_POWER_LSB_FACTOR 25.0f
#define INA226_SHUNT_RAW_POS_FULL_SCALE 0x7FFF
#define INA226_SHUNT_RAW_NEG_FULL_SCALE 0x8000
#define INA226_SHUNT_NOISE_RAW_DEADBAND 2
#define INA226_INTERNAL_LOG_INTERVAL_MS 1000

static const char *TAG = "ina226";
static uint32_t s_last_internal_log_ms[128] = {0};

static esp_err_t ina226_read_u16(uint8_t i2c_address, uint8_t reg, uint16_t *value)
{
    if (value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t rx[2] = {0};
    esp_err_t err = i2c_bus_write_read(
        i2c_address,
        &reg,
        1,
        rx,
        sizeof(rx),
        pdMS_TO_TICKS(50));
    if (err != ESP_OK) {
        return err;
    }

    *value = ((uint16_t)rx[0] << 8) | rx[1];
    return ESP_OK;
}

static esp_err_t ina226_write_u16(uint8_t i2c_address, uint8_t reg, uint16_t value)
{
    uint8_t tx[3] = {
        reg,
        (uint8_t)(value >> 8),
        (uint8_t)(value & 0xFF),
    };

    return i2c_bus_write(i2c_address, tx, sizeof(tx), pdMS_TO_TICKS(50));
}

esp_err_t ina226_init(uint8_t i2c_address, float shunt_resistance_ohm)
{
    esp_err_t err = i2c_bus_probe(i2c_address, pdMS_TO_TICKS(50));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "INA226 not responding at 0x%02X: %s", i2c_address, esp_err_to_name(err));
        return err;
    }

    uint16_t manufacturer_id = 0;
    uint16_t die_id = 0;

    err = ina226_read_u16(i2c_address, INA226_REG_MANUFACTURER_ID, &manufacturer_id);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read INA226 manufacturer id: %s", esp_err_to_name(err));
        return err;
    }

    err = ina226_read_u16(i2c_address, INA226_REG_DIE_ID, &die_id);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read INA226 die id: %s", esp_err_to_name(err));
        return err;
    }

    if (manufacturer_id != INA226_EXPECTED_MANUFACTURER_ID || die_id != INA226_EXPECTED_DIE_ID) {
        ESP_LOGW(
            TAG,
            "Unexpected INA226 IDs at 0x%02X (manuf=0x%04X die=0x%04X)",
            i2c_address,
            manufacturer_id,
            die_id);
    } else {
        ESP_LOGI(TAG, "INA226 detected at 0x%02X", i2c_address);
    }

    err = ina226_write_u16(i2c_address, INA226_REG_CONFIG, INA226_CONFIG_DEFAULT);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure INA226 at 0x%02X: %s", i2c_address, esp_err_to_name(err));
        return err;
    }

    // Set the calibration register so the INA226's internal current and power
    // registers can be used directly. Current_LSB is sized for a 1.0A full-scale
    // range, which keeps the register-based readings meaningful for this project.
    uint16_t calibration = (uint16_t)(0.00512f / (INA226_CURRENT_LSB_A * shunt_resistance_ohm));
    err = ina226_write_u16(i2c_address, INA226_REG_CALIBRATION, calibration);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set INA226 calibration at 0x%02X: %s", i2c_address, esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG,
             "INA226 calibration set to 0x%04X (Current_LSB=%.9fA, R_shunt=%.1fΩ)",
             calibration,
             INA226_CURRENT_LSB_A,
             shunt_resistance_ohm);
    return ESP_OK;
}

esp_err_t ina226_read_measurement(uint8_t i2c_address, float shunt_resistance_ohm, ina226_measurement_t *measurement)
{
    if (measurement == NULL || shunt_resistance_ohm <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t shunt_raw_u16 = 0;
    uint16_t bus_raw_u16 = 0;
    uint16_t current_raw_u16 = 0;
    uint16_t power_raw_u16 = 0;

    esp_err_t err = ina226_read_u16(i2c_address, INA226_REG_SHUNT_VOLTAGE, &shunt_raw_u16);
    if (err != ESP_OK) {
        return err;
    }

    err = ina226_read_u16(i2c_address, INA226_REG_BUS_VOLTAGE, &bus_raw_u16);
    if (err != ESP_OK) {
        return err;
    }

    err = ina226_read_u16(i2c_address, INA226_REG_CURRENT, &current_raw_u16);
    if (err != ESP_OK) {
        return err;
    }

    err = ina226_read_u16(i2c_address, INA226_REG_POWER, &power_raw_u16);
    if (err != ESP_OK) {
        return err;
    }

    if (shunt_raw_u16 == INA226_SHUNT_RAW_NEG_FULL_SCALE || shunt_raw_u16 == INA226_SHUNT_RAW_POS_FULL_SCALE) {
        ESP_LOGW(
            TAG,
            "INA226 shunt reading saturated at 0x%02X (raw=0x%04X); check shunt polarity/wiring/range",
            i2c_address,
            shunt_raw_u16);
        return ESP_ERR_INVALID_RESPONSE;
    }

    int16_t shunt_raw = (int16_t)shunt_raw_u16;
    if (shunt_raw >= -INA226_SHUNT_NOISE_RAW_DEADBAND && shunt_raw <= INA226_SHUNT_NOISE_RAW_DEADBAND) {
        shunt_raw = 0;
    }

    measurement->shunt_voltage_v = (float)shunt_raw * INA226_SHUNT_VOLTAGE_LSB_V;
    measurement->bus_voltage_v = (float)bus_raw_u16 * INA226_BUS_VOLTAGE_LSB_V;
    measurement->raw_current_s16 = (int16_t)current_raw_u16;
    measurement->raw_power_u16 = power_raw_u16;
    measurement->current_a = (float)measurement->raw_current_s16 * INA226_CURRENT_LSB_A;
    measurement->power_w = (float)measurement->raw_power_u16 * (INA226_POWER_LSB_FACTOR * INA226_CURRENT_LSB_A);
    measurement->raw_shunt_u16 = shunt_raw_u16;
    measurement->raw_bus_u16 = bus_raw_u16;

    uint32_t now_ms = esp_log_timestamp();
    if ((now_ms - s_last_internal_log_ms[i2c_address]) >= INA226_INTERNAL_LOG_INTERVAL_MS) {
        s_last_internal_log_ms[i2c_address] = now_ms;
        ESP_LOGI(
            TAG,
            "INA226 0x%02X | CAL=0x%04X | CUR_RAW=0x%04X (%d) -> %.6fA | PWR_RAW=0x%04X -> %.6fW",
            i2c_address,
            (uint16_t)(0.00512f / (INA226_CURRENT_LSB_A * shunt_resistance_ohm)),
            current_raw_u16,
            measurement->raw_current_s16,
            measurement->current_a,
            power_raw_u16,
            measurement->power_w);
    }

    return ESP_OK;
}
