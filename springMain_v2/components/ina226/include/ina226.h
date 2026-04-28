#pragma once

#include <stdint.h>

#include "esp_err.h"

#define INA226_I2C_ADDR_DEFAULT 0x40

typedef struct {
    float shunt_voltage_v;
    float bus_voltage_v;
    float current_a;
    float power_w;
    int16_t raw_current_s16;
    uint16_t raw_power_u16;
    uint16_t raw_shunt_u16;
    uint16_t raw_bus_u16;
} ina226_measurement_t;

esp_err_t ina226_init(uint8_t i2c_address, float shunt_resistance_ohm);
esp_err_t ina226_read_measurement(uint8_t i2c_address, float shunt_resistance_ohm, ina226_measurement_t *measurement);
