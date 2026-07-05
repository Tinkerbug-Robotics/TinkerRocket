idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES
        TR_Compat
        driver
        esp_timer
        esp_app_format
        TR_UART_Link
        TR_LoRa_Comms
)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error -Wno-format)
