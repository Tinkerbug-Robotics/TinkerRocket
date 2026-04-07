idf_component_register(SRCS "TR_GNSSReceiverUBlox_Serial.cpp" "u-blox_GNSS.cpp" "sfe_bus.cpp" INCLUDE_DIRS "." REQUIRES TR_Compat TR_RocketComputerTypes driver)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error -Wno-reorder -Wno-format-truncation -Wno-format)
