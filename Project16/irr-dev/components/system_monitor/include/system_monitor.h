#pragma once

typedef enum {
    SYSTEM_OK,
    SYSTEM_COOLDOWN,
    SYSTEM_PUMP_FAILURE,
    SYSTEM_SENSOR_ERROR,
    SYSTEM_CRITICAL_ERROR,
    SYSTEM_SHUTDOWN
} system_state_t;

typedef enum {
    ERROR_NONE,
    ERROR_TEMP_CRITICAL,
    ERROR_CURRENT_HIGH,
    ERROR_SENSOR_FAULT,
    ERROR_I2C_FAILURE,
    ERROR_PUMP_BLOCKED,
    ERROR_LOW_BATTERY
} error_code_t;

/**
 * Initialize system monitor
 */
void system_monitor_init(void);

/**
 * Get current system state
 */
system_state_t system_monitor_get_state(void);

/**
 * Check system health and update state
 */
void system_monitor_update(float temp1, float temp2, float current, float power);

/**
 * Enter cooldown mode
 */
void system_enter_cooldown(float current_temp, float target_temp);

/**
 * Check if system is in cooldown
 */
int system_is_in_cooldown(void);

/**
 * Check if system is safe to operate
 */
int system_is_safe(void);

/**
 * Log error with code
 */
void system_log_error(error_code_t code, const char *message);

/**
 * Get last error
 */
error_code_t system_get_last_error(void);

/**
 * Emergency shutdown
 */
void system_emergency_shutdown(const char *reason);

/**
 * Reset system to initial state
 */
void system_reset(void);
