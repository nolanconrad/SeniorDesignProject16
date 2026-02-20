#include "driver/i2c_master.h"

//setting up the master bus
i2c_master_bus_config_t i2c_master_bus_config = {
    .i2c_port = I2C_NUM_0, //check this
    .scl_io_num = 6,
    .sda_io_num = 5,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .glitch_ignore_cnt = 0, //check this
    .intr_priority = 0, 
    .enable_internal_pullup = true,
    .allow_pd_sleep = false, //could deal with this later, but probs not right now. Its about power consumption
};

//setting up the slave device
i2c_device_config_t i2c_device_config = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7, //bit length of the slave device
    .device_address = 0x4A, //I think this is correct based on the data sheet
    .scl_speed_hz = 100000, //100KHz is standard mode
    .scl_wait_us = 0, //this is the default time, and is whatever the driver configured as a safe deefault.
};

    //initializing the master bus
    i2c_new_master_bus(&i2c_master_bus_config);
    i2c_master_bus_handle_t bus_handle;
    i2c_master_bus_add_device(&i2c_master_bus_config, &bus_handle); //edit what goes in here


    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_master_bus_config, &bus_handle));
    ESP_ERROR_CHECK(i2c_master_get_bus_handle(0, &handle));

    const uint8_t *write_buffer;
    uint8_t data_to_send[2] = {0x00, 0x01}; //example data to send

    uint8_t read_buffer[2]; //buffer to store received data

    i2c_master_transmit(bus_handle, data_to_send, 2, 1000); //arrays automativelally decay to pointers
    i2c_master_receive(bus_handle, read_buffer, 2, 1000);