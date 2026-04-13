#pragma once

#include "driver/i2c_master.h"

void tmp117_init(void);
float tmp117_read_temperature(void);
float tmp117_read_temperature_device2(void);
void tmp117_register_error_callback(void (*callback)(int device));
void tmp117_register_overheat_callback(void (*callback)(int device, float temperature));

i2c_master_bus_handle_t get_i2c_bus_handle(void);