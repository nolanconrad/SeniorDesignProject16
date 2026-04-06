idf_component_register(SRCS "pump_Operation.c"
                       INCLUDE_DIRS "include"
                       REQUIRES freertos
                       PRIV_REQUIRES driver)
