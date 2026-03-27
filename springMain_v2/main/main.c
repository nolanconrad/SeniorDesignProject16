#include <stdbool.h>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "i2c_bus.h"
#include "ina226.h"
#include "pumpOperation.h"
#include "tmp117.h"

static const char *TAG = "main";

// Button pin definitions (active-low: button shorts pin to GND when pressed)
#define BUTTON_D2_PIN GPIO_NUM_5
#define BUTTON_A2_PIN GPIO_NUM_3
#define BUTTON_A3_PIN GPIO_NUM_4

// LED pin definitions for D3..D9.
#define LED_D3_PIN GPIO_NUM_6
#define LED_D4_PIN GPIO_NUM_7
#define LED_D5_PIN GPIO_NUM_8
#define LED_D6_PIN GPIO_NUM_9
#define LED_D7_PIN GPIO_NUM_10
#define LED_D8_PIN GPIO_NUM_17
#define LED_D9_PIN GPIO_NUM_18

#define LED_D6_BLINK_PERIOD_MS 500
#define LED_SEQUENCE_STEP_MS 400
#define PUMP_CYCLE_ON_MS 2000
#define PUMP_CYCLE_OFF_MS 2000
#define PUMP_CYCLE_ON_DUTY_PERCENT 50

#define TMP117_SENSOR_1_ADDR 0x48
#define INA226_SENSOR_1_ADDR 0x40
#define INA226_SENSOR_2_ADDR 0x41
#define INA226_SHUNT_RESISTANCE_OHM 0.1f
#define TMP117_READ_PERIOD_MS 1000

typedef struct {
	const char *name;
	uint8_t i2c_address;
	bool initialized;
} tmp117_sensor_t;

typedef struct {
	const char *name;
	uint8_t i2c_address;
	float shunt_resistance_ohm;
	bool initialized;
} ina226_sensor_t;

// Button state tracking - simple version
typedef struct {
	const char *button_name;
	gpio_num_t gpio_num;
	int last_level;
	gpio_num_t controlled_led_pin;
	int controlled_led_state;
} button_state_t;

static button_state_t button_states[] = {
	{.button_name = "D2", .gpio_num = BUTTON_D2_PIN, .last_level = 1, .controlled_led_pin = LED_D7_PIN, .controlled_led_state = 0},
	{.button_name = "A2", .gpio_num = BUTTON_A2_PIN, .last_level = 1, .controlled_led_pin = LED_D8_PIN, .controlled_led_state = 0},
	{.button_name = "A3", .gpio_num = BUTTON_A3_PIN, .last_level = 1, .controlled_led_pin = LED_D9_PIN, .controlled_led_state = 0},
};

static const gpio_num_t led_pins[] = {
	LED_D3_PIN,
	LED_D4_PIN,
	LED_D5_PIN,
	LED_D6_PIN,
	LED_D7_PIN,
	LED_D8_PIN,
	LED_D9_PIN,
};

static const gpio_num_t led_sequence_pins[] = {
	LED_D3_PIN,
	LED_D4_PIN,
	LED_D5_PIN,
};

// Initialize button GPIO pins
static esp_err_t buttons_init(void)
{
	// Use internal pull-ups for stable button-to-ground wiring.
	gpio_config_t btn_gpio_cfg = {
		.pin_bit_mask = (1ULL << BUTTON_D2_PIN) | (1ULL << BUTTON_A2_PIN) | (1ULL << BUTTON_A3_PIN),
		.mode = GPIO_MODE_INPUT,
		.pull_up_en = GPIO_PULLUP_ENABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE,  // No interrupts; we'll poll instead
	};

	esp_err_t err = gpio_config(&btn_gpio_cfg);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Failed to configure button GPIOs: %s", esp_err_to_name(err));
		return err;
	}

	ESP_LOGI(TAG, "Button GPIO configured (active-low with pull-ups) on D2 (GPIO%d), A2 (GPIO%d), A3 (GPIO%d)", 
		BUTTON_D2_PIN, BUTTON_A2_PIN, BUTTON_A3_PIN);

	for (int i = 0; i < (int)(sizeof(button_states) / sizeof(button_states[0])); i++) {
		button_states[i].last_level = gpio_get_level(button_states[i].gpio_num);
	}
	return ESP_OK;
}

// Task to poll buttons - simple digital read with press edge detection
static void button_poll_task(void *arg)
{
	(void)arg;
	const int num_buttons = sizeof(button_states) / sizeof(button_states[0]);
	
	while (1) {
		for (int i = 0; i < num_buttons; i++) {
			button_state_t *btn = &button_states[i];
			int level = gpio_get_level(btn->gpio_num);

			// Active-low press: HIGH -> LOW transition means button pressed.
			if (btn->last_level == 1 && level == 0) {
				btn->controlled_led_state = !btn->controlled_led_state;
				gpio_set_level(btn->controlled_led_pin, btn->controlled_led_state);
				ESP_LOGI(TAG, "Button %s pressed -> LED GPIO%d %s",
					btn->button_name,
					btn->controlled_led_pin,
					btn->controlled_led_state ? "ON" : "OFF");
			}

			btn->last_level = level;
		}
		
		vTaskDelay(pdMS_TO_TICKS(20));
	}
}

static esp_err_t leds_init(void)
{
	uint64_t pin_mask = 0;
	for (int i = 0; i < (int)(sizeof(led_pins) / sizeof(led_pins[0])); i++) {
		pin_mask |= (1ULL << led_pins[i]);
	}

	gpio_config_t led_gpio_cfg = {
		.pin_bit_mask = pin_mask,
		.mode = GPIO_MODE_OUTPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE,
	};

	esp_err_t err = gpio_config(&led_gpio_cfg);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Failed to configure LED GPIOs: %s", esp_err_to_name(err));
		return err;
	}

	for (int i = 0; i < (int)(sizeof(led_pins) / sizeof(led_pins[0])); i++) {
		gpio_set_level(led_pins[i], 0);
	}

	ESP_LOGI(TAG, "LED outputs initialized on D3..D9");
	return ESP_OK;
}

static void d6_blink_task(void *arg)
{
	(void)arg;
	int level = 0;

	while (1) {
		level = !level;
		gpio_set_level(LED_D6_PIN, level);
		vTaskDelay(pdMS_TO_TICKS(LED_D6_BLINK_PERIOD_MS));
	}
}

static void led_sequence_task(void *arg)
{
	(void)arg;

	while (1) {
		gpio_set_level(led_sequence_pins[0], 1);
		gpio_set_level(led_sequence_pins[1], 0);
		gpio_set_level(led_sequence_pins[2], 0);
		vTaskDelay(pdMS_TO_TICKS(LED_SEQUENCE_STEP_MS));

		gpio_set_level(led_sequence_pins[0], 1);
		gpio_set_level(led_sequence_pins[1], 1);
		gpio_set_level(led_sequence_pins[2], 0);
		vTaskDelay(pdMS_TO_TICKS(LED_SEQUENCE_STEP_MS));

		gpio_set_level(led_sequence_pins[0], 1);
		gpio_set_level(led_sequence_pins[1], 1);
		gpio_set_level(led_sequence_pins[2], 1);
		vTaskDelay(pdMS_TO_TICKS(LED_SEQUENCE_STEP_MS));

		gpio_set_level(led_sequence_pins[0], 0);
		gpio_set_level(led_sequence_pins[1], 0);
		gpio_set_level(led_sequence_pins[2], 0);
		vTaskDelay(pdMS_TO_TICKS(LED_SEQUENCE_STEP_MS));
	}
}

static void temp_monitor_task(void *arg)
{
	(void)arg;

	static tmp117_sensor_t tmp_sensor = {
		.name = "tmp117_1",
		.i2c_address = TMP117_SENSOR_1_ADDR,
		.initialized = false,
	};

	static ina226_sensor_t current_sensors[] = {
		{.name = "ina226_1", .i2c_address = INA226_SENSOR_1_ADDR, .shunt_resistance_ohm = INA226_SHUNT_RESISTANCE_OHM, .initialized = false},
		{.name = "ina226_2", .i2c_address = INA226_SENSOR_2_ADDR, .shunt_resistance_ohm = INA226_SHUNT_RESISTANCE_OHM, .initialized = false},
	};

	const size_t current_sensor_count = sizeof(current_sensors) / sizeof(current_sensors[0]);
	TickType_t last_wake = xTaskGetTickCount();

	while (1) {
		if (!tmp_sensor.initialized) {
			esp_err_t init_err = tmp117_init(tmp_sensor.i2c_address);
			if (init_err == ESP_OK) {
				tmp_sensor.initialized = true;
				ESP_LOGI(TAG, "%s initialized at 0x%02X", tmp_sensor.name, tmp_sensor.i2c_address);
			} else {
				ESP_LOGW(TAG, "%s init failed at 0x%02X: %s", tmp_sensor.name, tmp_sensor.i2c_address, esp_err_to_name(init_err));
			}
		}

		if (tmp_sensor.initialized) {
			float temperature_c = 0.0f;
			esp_err_t read_err = tmp117_read_temperature_c(tmp_sensor.i2c_address, &temperature_c);
			if (read_err == ESP_OK) {
				ESP_LOGI(TAG, "%s (0x%02X): %.3f C", tmp_sensor.name, tmp_sensor.i2c_address, temperature_c);
			} else {
				ESP_LOGW(TAG, "%s read failed at 0x%02X: %s", tmp_sensor.name, tmp_sensor.i2c_address, esp_err_to_name(read_err));
				tmp_sensor.initialized = false;
			}
		}

		for (size_t i = 0; i < current_sensor_count; ++i) {
			ina226_sensor_t *sensor = &current_sensors[i];

			if (!sensor->initialized) {
				esp_err_t init_err = ina226_init(sensor->i2c_address);
				if (init_err == ESP_OK) {
					sensor->initialized = true;
					ESP_LOGI(TAG, "%s initialized at 0x%02X", sensor->name, sensor->i2c_address);
				} else {
					ESP_LOGW(TAG, "%s init failed at 0x%02X: %s", sensor->name, sensor->i2c_address, esp_err_to_name(init_err));
					continue;
				}
			}

			ina226_measurement_t measurement = {0};
			esp_err_t read_err = ina226_read_measurement(
				sensor->i2c_address,
				sensor->shunt_resistance_ohm,
				&measurement);
			if (read_err == ESP_OK) {
				ESP_LOGI(
					TAG,
					"%s (0x%02X): I=%.4f A, Vbus=%.3f V, Vshunt=%.5f V, P=%.3f W",
					sensor->name,
					sensor->i2c_address,
					measurement.current_a,
					measurement.bus_voltage_v,
					measurement.shunt_voltage_v,
					measurement.power_w);
			} else {
				ESP_LOGW(TAG, "%s read failed at 0x%02X: %s", sensor->name, sensor->i2c_address, esp_err_to_name(read_err));
				sensor->initialized = false;
			}
		}

		vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TMP117_READ_PERIOD_MS));
	}
}

static void pump_cycle_task(void *arg)
{
	(void)arg;

	while (1) {
		esp_err_t on_err = pump_operation_set_duty_percent(PUMP_CYCLE_ON_DUTY_PERCENT);
		if (on_err != ESP_OK) {
			ESP_LOGW(TAG, "Failed to set pump duty to %d%%: %s", PUMP_CYCLE_ON_DUTY_PERCENT, esp_err_to_name(on_err));
		}
		vTaskDelay(pdMS_TO_TICKS(PUMP_CYCLE_ON_MS));

		esp_err_t off_err = pump_operation_stop();
		if (off_err != ESP_OK) {
			ESP_LOGW(TAG, "Failed to stop pump: %s", esp_err_to_name(off_err));
		}
		vTaskDelay(pdMS_TO_TICKS(PUMP_CYCLE_OFF_MS));
	}
}

void app_main(void)
{
	esp_err_t err = i2c_bus_init();
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "I2C init failed: %s", esp_err_to_name(err));
		return;
	}

	// Initialize buttons
	err = buttons_init();
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Button initialization failed: %s", esp_err_to_name(err));
		return;
	}

	err = leds_init();
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "LED initialization failed: %s", esp_err_to_name(err));
		return;
	}

	err = pump_operation_init();
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Pump driver initialization failed: %s", esp_err_to_name(err));
		return;
	}

	err = pump_operation_stop();
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Failed to set pump output to 0%%: %s", esp_err_to_name(err));
		return;
	}

	// Start button polling task
	BaseType_t btn_task_ok = xTaskCreate(button_poll_task, "button_poll_task", 2048, NULL, 4, NULL);
	if (btn_task_ok != pdPASS) {
		ESP_LOGE(TAG, "Failed to create button poll task");
		return;
	}

	BaseType_t d6_task_ok = xTaskCreate(d6_blink_task, "d6_blink_task", 2048, NULL, 4, NULL);
	if (d6_task_ok != pdPASS) {
		ESP_LOGE(TAG, "Failed to create D6 blink task");
		return;
	}

	BaseType_t seq_task_ok = xTaskCreate(led_sequence_task, "led_sequence_task", 2048, NULL, 4, NULL);
	if (seq_task_ok != pdPASS) {
		ESP_LOGE(TAG, "Failed to create LED sequence task");
		return;
	}

	ESP_LOGI(TAG, "I2C bus initialized. Scanning bus for connected devices...");
	i2c_bus_scan(pdMS_TO_TICKS(500));

	BaseType_t task_ok = xTaskCreate(temp_monitor_task, "temp_monitor_task", 4096, NULL, 5, NULL);
	if (task_ok != pdPASS) {
		ESP_LOGE(TAG, "Failed to create temperature monitor task");
		return;
	}

	BaseType_t pump_task_ok = xTaskCreate(pump_cycle_task, "pump_cycle_task", 2048, NULL, 4, NULL);
	if (pump_task_ok != pdPASS) {
		ESP_LOGE(TAG, "Failed to create pump cycle task");
		return;
	}

	ESP_LOGI(TAG, "Temperature/current monitor task started for one TMP117 and two INA226 sensors.");
	ESP_LOGI(TAG, "Pump cycle task started: %d%% for %d ms, then OFF for %d ms.",
		PUMP_CYCLE_ON_DUTY_PERCENT,
		PUMP_CYCLE_ON_MS,
		PUMP_CYCLE_OFF_MS);
}
