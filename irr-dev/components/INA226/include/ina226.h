#pragma once

#include <stdint.h>

#define INA226_DEVICE_PRIMARY 0
#define INA226_DEVICE_SECONDARY 1
#define INA226_DEVICE_LOGIC INA226_DEVICE_SECONDARY

/**
 * Initialize INA226 power monitor
 * @return 0 on success, non-zero on error
 */
int ina226_init(void);

/**
 * Recalibrate zero-current offset while load is known to be off
 * @return 0 on success, non-zero on error
 */
int ina226_recalibrate_zero_offset(void);

/**
 * Recalibrate zero-current offset for a specific INA226 device index
 * @param device_index 0 for address 0x40, 1 for address 0x41
 * @return 0 on success, non-zero on error
 */
int ina226_recalibrate_zero_offset_device(uint8_t device_index);

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
 * Read bus voltage (V) from a specific INA226 device
 * @param device_index 0 for address 0x40, 1 for address 0x41
 * @return Voltage in volts
 */
float ina226_read_bus_voltage_device(uint8_t device_index);

/**
 * Read shunt voltage (mV) from a specific INA226 device
 * @param device_index 0 for address 0x40, 1 for address 0x41
 * @return Voltage in millivolts
 */
float ina226_read_shunt_voltage_device(uint8_t device_index);

/**
 * Read current (A) from a specific INA226 device
 * @param device_index 0 for address 0x40, 1 for address 0x41
 * @return Current in amperes
 */
float ina226_read_current_device(uint8_t device_index);

/**
 * Read power (W) from a specific INA226 device
 * @param device_index 0 for address 0x40, 1 for address 0x41
 * @return Power in watts
 */
float ina226_read_power_device(uint8_t device_index);

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
