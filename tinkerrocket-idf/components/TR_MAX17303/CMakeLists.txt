idf_component_register(SRCS "TR_MAX17303.cpp" INCLUDE_DIRS "." REQUIRES esp_driver_i2c TR_Compat PRIV_REQUIRES driver)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error -Wno-reorder -Wno-format-truncation -Wno-format)
