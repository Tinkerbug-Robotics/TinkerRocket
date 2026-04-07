idf_component_register(
    SRCS "TR_I2S_Stream.cpp"
    INCLUDE_DIRS "."
    REQUIRES TR_Compat TR_I2C_Interface TR_RocketComputerTypes
    PRIV_REQUIRES driver esp_driver_i2s esp_driver_gpio
)
