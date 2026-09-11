idf_component_register(SRCS "TR_QMC5883P.cpp" INCLUDE_DIRS "." REQUIRES esp_driver_i2c TR_Compat PRIV_REQUIRES driver)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error -Wno-reorder -Wno-format-truncation -Wno-format)
