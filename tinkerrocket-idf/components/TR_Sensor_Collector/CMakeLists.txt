idf_component_register(SRCS "TR_Sensor_Collector.cpp" INCLUDE_DIRS "." REQUIRES TR_Compat TR_RocketComputerTypes TR_ISM6HG256 TR_BMP585 TR_MMC5983MA TR_GNSSReceiverUBlox_Serial)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error -Wno-reorder -Wno-format-truncation -Wno-format)
