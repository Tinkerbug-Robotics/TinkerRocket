idf_component_register(SRCS "TR_LoRa_Comms.cpp" INCLUDE_DIRS "." REQUIRES esp_driver_gpio esp_driver_spi RadioLib TR_Compat driver esp_timer)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error -Wno-reorder -Wno-format-truncation -Wno-format)
