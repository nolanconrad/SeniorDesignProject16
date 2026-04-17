#include "i2c_bus.h"

#include <stdbool.h>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define I2C_MASTER_NUM I2C_NUM_0
// Arduino Nano ESP32 (ESP32-S3) wiring used here: A5=SCL, A4=SDA.
#define I2C_MASTER_SDA_IO GPIO_NUM_11 // A4
#define I2C_MASTER_SCL_IO GPIO_NUM_12 // A5
#define I2C_MASTER_FREQ_HZ 100000

static const char *TAG = "i2c_bus";
static bool s_initialized = false;

esp_err_t i2c_bus_init(void)
{
	if (s_initialized) {
		return ESP_OK;
	}

	i2c_config_t conf = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = I2C_MASTER_SDA_IO,
		.scl_io_num = I2C_MASTER_SCL_IO,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = I2C_MASTER_FREQ_HZ,
		.clk_flags = 0,
	};

	esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "i2c_param_config failed: %s", esp_err_to_name(err));
		return err;
	}

	err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "i2c_driver_install failed: %s", esp_err_to_name(err));
		return err;
	}

	s_initialized = true;
	ESP_LOGI(TAG, "I2C master ready on port %d (SDA=%d, SCL=%d, %d Hz)",
			 I2C_MASTER_NUM, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, I2C_MASTER_FREQ_HZ);

	return ESP_OK;
}

esp_err_t i2c_bus_probe(uint8_t device_address, TickType_t timeout_ticks)
{
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	if (cmd == NULL) {
		return ESP_ERR_NO_MEM;
	}

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (device_address << 1) | I2C_MASTER_WRITE, true);
	i2c_master_stop(cmd);

	esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, timeout_ticks);
	i2c_cmd_link_delete(cmd);
	return err;
}

esp_err_t i2c_bus_write(uint8_t device_address, const uint8_t *data, size_t data_len, TickType_t timeout_ticks)
{
	if (data == NULL || data_len == 0) {
		return ESP_ERR_INVALID_ARG;
	}

	return i2c_master_write_to_device(I2C_MASTER_NUM, device_address, data, data_len, timeout_ticks);
}

esp_err_t i2c_bus_write_read(
	uint8_t device_address,
	const uint8_t *write_data,
	size_t write_len,
	uint8_t *read_data,
	size_t read_len,
	TickType_t timeout_ticks)
{
	if (write_data == NULL || write_len == 0 || read_data == NULL || read_len == 0) {
		return ESP_ERR_INVALID_ARG;
	}

	return i2c_master_write_read_device(
		I2C_MASTER_NUM,
		device_address,
		write_data,
		write_len,
		read_data,
		read_len,
		timeout_ticks);
}

void i2c_bus_scan(TickType_t timeout_ticks)
{
	bool found_any = false;

	for (uint8_t addr = 1; addr < 0x7F; ++addr) {
		esp_err_t err = i2c_bus_probe(addr, timeout_ticks);
		if (err == ESP_OK) {
			found_any = true;
			ESP_LOGI(TAG, "I2C device found at 0x%02X", addr);
		}
	}

	if (!found_any) {
		ESP_LOGW(TAG, "No I2C devices found. Check wiring, power, and pull-ups.");
	}
}
