idf_component_register(SRCS "TR_Sensor_Data_Converter.cpp" INCLUDE_DIRS "." REQUIRES TR_Compat TR_RocketComputerTypes)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error -Wno-reorder -Wno-format-truncation -Wno-format)
