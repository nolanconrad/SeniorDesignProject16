#pragma once

#include <stdint.h>

/**
 * Initialize INA226 power monitor
 * @return 0 on success, non-zero on error
 */
int ina226_init(void);

/**
 * Read bus voltage (V)
 * @return Voltage in volts
 */
float ina226_read_bus_voltage(void);

/**
 * Read shunt voltage (mV)
 * @return Voltage in millivolts
 */
float ina226_read_shunt_voltage(void);

/**
 * Read current (A)
 * @return Current in amperes
 */
float ina226_read_current(void);

/**
 * Read power (W)
 * @return Power in watts
 */
float ina226_read_power(void);

/**
 * Register callback for low power alert
 * @param callback Function called when power drops below threshold
 */
void ina226_register_low_power_callback(void (*callback)(float power));

/**
 * Register callback for high current alert
 * @param callback Function called when current exceeds threshold
 */
void ina226_register_high_current_callback(void (*callback)(float current));

/**
 * Set current alarm threshold (A)
 * @param threshold Current threshold in amperes
 */
void ina226_set_current_threshold(float threshold);

/**
 * Set power alarm threshold (W)
 * @param threshold Power threshold in watts
 */
void ina226_set_power_threshold(float threshold);
