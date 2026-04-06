idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES system_startUp
                                     i2c_with_tmp117
                                     INA226
                                     system_monitor
                                     startup_diagnostic
                                     error_handler)
