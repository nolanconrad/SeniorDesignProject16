#include "tmp117.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "i2c_bus.h"

#define TMP117_REG_TEMP_RESULT 0x00
#define TMP117_REG_DEVICE_ID 0x0F
#define TMP117_EXPECTED_DEVICE_ID 0x0117
#define TMP117_TEMP_LSB_C 0.0078125f

static const char *TAG = "tmp117";

static esp_err_t tmp117_read_u16(uint8_t i2c_address, uint8_t reg, uint16_t *value)
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

esp_err_t tmp117_init(uint8_t i2c_address)
{
    esp_err_t err = i2c_bus_probe(i2c_address, pdMS_TO_TICKS(50));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TMP117 not responding at 0x%02X: %s", i2c_address, esp_err_to_name(err));
        return err;
    }

    uint16_t device_id = 0;
    err = tmp117_read_u16(i2c_address, TMP117_REG_DEVICE_ID, &device_id);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read TMP117 device id: %s", esp_err_to_name(err));
        return err;
    }

    if (device_id != TMP117_EXPECTED_DEVICE_ID) {
        ESP_LOGW(TAG, "Unexpected TMP117 device id 0x%04X (expected 0x%04X)", device_id, TMP117_EXPECTED_DEVICE_ID);
    } else {
        ESP_LOGI(TAG, "TMP117 detected at 0x%02X (device id 0x%04X)", i2c_address, device_id);
    }

    return ESP_OK;
}

esp_err_t tmp117_read_temperature_c(uint8_t i2c_address, float *temperature_c)
{
    if (temperature_c == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t raw_u16 = 0;
    esp_err_t err = tmp117_read_u16(i2c_address, TMP117_REG_TEMP_RESULT, &raw_u16);
    if (err != ESP_OK) {
        return err;
    }

    int16_t raw_temp = (int16_t)raw_u16;
    *temperature_c = (float)raw_temp * TMP117_TEMP_LSB_C;
    return ESP_OK;
}
