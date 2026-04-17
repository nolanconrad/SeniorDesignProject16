#pragma once

#include <stdint.h>

#include "esp_err.h"

#define TMP117_I2C_ADDR_DEFAULT 0x48

esp_err_t tmp117_init(uint8_t i2c_address);
esp_err_t tmp117_read_temperature_c(uint8_t i2c_address, float *temperature_c);
