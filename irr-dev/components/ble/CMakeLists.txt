idf_component_register(
    SRCS "ble_handler.cpp"
    INCLUDE_DIRS "include"
    REQUIRES bt nvs_flash pump_Operation
)