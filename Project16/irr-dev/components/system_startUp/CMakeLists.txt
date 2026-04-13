idf_component_register(SRCS "system_startUp.c"
                       INCLUDE_DIRS "include"
                       REQUIRES freertos
                       PRIV_REQUIRES i2c_with_tmp117
                                     pump_Operation
                                     driver)
