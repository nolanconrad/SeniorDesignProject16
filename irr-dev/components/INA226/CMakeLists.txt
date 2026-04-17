idf_component_register(
    SRCS "INA226.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_i2c i2c_with_tmp117
)