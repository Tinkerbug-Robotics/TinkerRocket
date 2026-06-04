idf_component_register(SRCS "TR_ServoControl_ledc_mult.cpp" INCLUDE_DIRS "." REQUIRES esp_driver_ledc TR_Compat TR_PID PRIV_REQUIRES driver)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error -Wno-reorder -Wno-format-truncation -Wno-format)
