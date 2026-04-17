#pragma once

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

esp_err_t i2c_bus_init(void);
esp_err_t i2c_bus_probe(uint8_t device_address, TickType_t timeout_ticks);
esp_err_t i2c_bus_write(uint8_t device_address, const uint8_t *data, size_t data_len, TickType_t timeout_ticks);
esp_err_t i2c_bus_write_read(
	uint8_t device_address,
	const uint8_t *write_data,
	size_t write_len,
	uint8_t *read_data,
	size_t read_len,
	TickType_t timeout_ticks);
void i2c_bus_scan(TickType_t timeout_ticks);
