#pragma once

typedef struct {
    int pump_test_passed;
    int led_test_passed;
    int sensor_test_passed;
    int i2c_test_passed;
    int all_tests_passed;
} startup_test_results_t;

/**
 * Run full startup diagnostics
 * @return 1 if all tests pass, 0 if any fail
 */
int startup_diagnostic_run(void);

/**
 * Get diagnostic results
 */
startup_test_results_t startup_diagnostic_get_results(void);

/**
 * Run pump test
 */
int startup_test_pump(void);

/**
 * Run LED test
 */
int startup_test_leds(void);

/**
 * Run sensor test
 */
int startup_test_sensors(void);

/**
 * Run I2C communication test
 */
int startup_test_i2c(void);

/**
 * Display test results to user
 */
void startup_diagnostic_display_results(void);
