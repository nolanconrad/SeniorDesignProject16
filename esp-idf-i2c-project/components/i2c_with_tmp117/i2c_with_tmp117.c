#include "i2c_with_tmp117.h"
#include "freertos/FreeRTOS.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include <math.h>
#include <stdint.h>

static const char *TAG = "I2C_EXAMPLE";

static i2c_master_bus_handle_t bus_handle = NULL;
static i2c_master_dev_handle_t dev_handle = NULL;
static i2c_master_dev_handle_t dev_handle2 = NULL;

static float tmp117_read_temperature_from_device(i2c_master_dev_handle_t handle)
{
    static const uint8_t temp_reg = 0x00;
    uint8_t temp_raw[2] = {0};

    esp_err_t err = i2c_master_transmit_receive(
        handle,
        &temp_reg,
        1,
        temp_raw,
        2,
        pdMS_TO_TICKS(100));

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TMP117 read failed: %s", esp_err_to_name(err));
        return NAN;
    }

    int16_t raw_temp = (int16_t)((temp_raw[0] << 8) | temp_raw[1]);
    return raw_temp * 0.0078125f;
}

void tmp117_init(void)
{
    if (bus_handle != NULL) {
        return;
    }

    ESP_LOGI(TAG, "I2C Master initialization");

    //setting up the master bus
    i2c_master_bus_config_t i2c_master_bus_config = {
        .i2c_port = I2C_NUM_0, //check this
        .scl_io_num = 6,
        .sda_io_num = 5,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7, //I guess 7 is the default?
        .intr_priority = 0,
        .flags.enable_internal_pullup = true,
    };

    //setting up the first slave device (tmp117 with the eaddress pin low)
    i2c_device_config_t i2c_device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7, //bit length of the slave device
        .device_address = 0x48, //address is 0x48 when the eaddress pin is low (0x49 when the eaddress pin is high)
        .scl_speed_hz = 100000, //100KHz is standard mode
    };

    //setting up the second slave device (tmp117 with the eaddress pin high)
    i2c_device_config_t i2c_device_config2 = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7, //bit length of the slave device
        .device_address = 0x49, //
        .scl_speed_hz = 100000, //100KHz is standard mode
    };

    //initializing the master bus
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_master_bus_config, &bus_handle));

    //add device to the bus
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &i2c_device_config, &dev_handle));

    // Add second device
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &i2c_device_config2, &dev_handle2));
}

float tmp117_read_temperature(void)
{
    if (dev_handle == NULL) {
        ESP_LOGE(TAG, "TMP117 device 1 not initialized");
        return NAN;
    }

    return tmp117_read_temperature_from_device(dev_handle);
}

float tmp117_read_temperature_device2(void)
{
    if (dev_handle2 == NULL) {
        ESP_LOGE(TAG, "TMP117 device 2 not initialized");
        return NAN;
    }

    return tmp117_read_temperature_from_device(dev_handle2);
}

