idf_component_register(
    SRCS "ble_handler.c"
    INCLUDE_DIRS "include"
    REQUIRES bt nvs_flash pumpOperation
)