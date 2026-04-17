#pragma once

#include <stdint.h>

#include "esp_err.h"

esp_err_t pump_operation_init(void);
esp_err_t pump_operation_set_duty_percent(uint8_t duty_percent);
esp_err_t pump_operation_stop(void);
uint8_t pump_operation_get_duty_percent(void);
