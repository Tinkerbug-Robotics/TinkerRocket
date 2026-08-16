idf_component_register(SRCS "TR_Sensor_Collector.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_gpio esp_driver_i2c TR_RocketComputerTypes TR_ISM6HG256 TR_BMP585 TR_MMC5983MA TR_IIS2MDC TR_GNSSReceiverUBlox_Serial TR_GNSSReceiverLC86_Serial
                                driver esp_timer esp_rom)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error -Wno-reorder -Wno-format-truncation -Wno-format)
