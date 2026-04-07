idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES arduino TR_I2S_Stream TR_I2C_Interface TR_RocketComputerTypes CRC
)
