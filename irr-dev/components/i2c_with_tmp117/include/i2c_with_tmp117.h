#pragma once

void tmp117_init(void);
float tmp117_read_temperature(void);
float tmp117_read_temperature_device2(void);
void tmp117_register_error_callback(void (*callback)(int device));


