#include <stdio.h>
#include "i2c_with_tmp117.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

static const char *TAG = "I2C_EXAMPLE";

// Static handles for I2C communication
static i2c_master_bus_handle_t bus_handle = NULL;
static i2c_master_dev_handle_t dev_handle = NULL;
static i2c_master_dev_handle_t dev_handle2 = NULL;

void tmp117_init(void)
{
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
    
    ESP_LOGI(TAG, "I2C initialization complete");
}

float tmp117_read_temperature(void)
{
    uint8_t data_to_send[2] = {0x00, 0x01}; //example data to send
    uint8_t read_buffer1[2] = {0}; //buffer to store received data from device 1
    
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, data_to_send, 2, read_buffer1, 2, pdMS_TO_TICKS(1000)));
    ESP_LOGI(TAG, "Data transmitted to device 1, received: 0x%02X 0x%02X", read_buffer1[0], read_buffer1[1]);
    
    // TODO: Convert raw bytes to temperature value
    int16_t raw_temp = (read_buffer1[0] << 8) | read_buffer1[1]; //combine the two bytes into a single 16-bit value
    float temperature = raw_temp * 0.0078125f; //convert raw value to temperature in Celsius (TMP117 resolution is 0.0078125°C per LSB)
    return temperature;
}

float tmp117_read_temperature_device2(void)
{
    uint8_t data_to_send[2] = {0x00, 0x01}; //example data to send
    uint8_t read_buffer2[2] = {0}; //buffer to store received data from device 2
    
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle2, data_to_send, 2, read_buffer2, 2, pdMS_TO_TICKS(1000)));
    ESP_LOGI(TAG, "Data transmitted to device 2, received: 0x%02X 0x%02X", read_buffer2[0], read_buffer2[1]);
    
    // TODO: Convert raw bytes to temperature value
    int16_t raw_temp = (read_buffer2[0] << 8) | read_buffer2[1]; //combine the two bytes into a single 16-bit value
    float temperature = raw_temp * 0.0078125f; //convert raw value to temperature in Celsius (TMP117 resolution is 0.0078125°C per LSB)
    return temperature;
}

