idf_component_register(SRCS "TR_UART_Link.cpp" "TR_MsgCodec.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_uart driver CRC TR_Compat)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error)
