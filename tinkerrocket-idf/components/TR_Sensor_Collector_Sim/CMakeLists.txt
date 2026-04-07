idf_component_register(SRCS "TR_Sensor_Collector_Sim.cpp" INCLUDE_DIRS "." REQUIRES TR_Compat TR_RocketComputerTypes TR_Sensor_Collector)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error -Wno-reorder -Wno-format-truncation -Wno-format)
