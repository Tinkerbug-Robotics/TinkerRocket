idf_component_register(SRCS "UartModemBackend.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES TR_LoRa_Comms TR_UART_Link TR_Compat
                                esp_driver_gpio driver)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error)
