#pragma once

typedef enum {
    MODE_STARTUP,
    MODE_MANUAL_LOW,
    MODE_MANUAL_MEDIUM,
    MODE_MANUAL_HIGH,
    MODE_AUTOMATIC,
    MODE_ERROR,
    MODE_COOLDOWN
} operating_mode_t;

/**
 * Initialize operation mode manager
 */
void mode_manager_init(void);

/**
 * Set operating mode
 */
void mode_manager_set_mode(operating_mode_t mode);

/**
 * Get current mode
 */
operating_mode_t mode_manager_get_mode(void);

/**
 * Check if automatic mode is enabled by user
 */
int mode_manager_is_automatic_enabled(void);

/**
 * Set automatic mode enable flag
 */
void mode_manager_set_automatic_enabled(int enabled);

/**
 * Get manual mode level (1, 2, 3)
 */
int mode_manager_get_manual_level(void);

/**
 * Cycle manual mode level
 */
void mode_manager_cycle_manual_level(void);

/**
 * Handle mode transition with safety checks
 */
int mode_manager_transition(operating_mode_t new_mode);
