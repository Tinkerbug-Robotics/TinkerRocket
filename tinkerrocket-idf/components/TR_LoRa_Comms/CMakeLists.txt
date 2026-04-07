idf_component_register(SRCS "TR_LoRa_Comms.cpp" INCLUDE_DIRS "." REQUIRES RadioLib TR_Compat driver esp_timer)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error -Wno-reorder -Wno-format-truncation -Wno-format)
