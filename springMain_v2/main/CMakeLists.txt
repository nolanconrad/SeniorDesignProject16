idf_component_register(
	SRCS "main.c"
	INCLUDE_DIRS "."
	REQUIRES i2c tmp117 ina226 pumpOperation
)
