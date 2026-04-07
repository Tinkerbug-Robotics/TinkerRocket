idf_component_register(SRCS "TR_I2C_Interface.cpp" INCLUDE_DIRS "." REQUIRES TR_Compat TR_RocketComputerTypes CRC PRIV_REQUIRES driver)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error -Wno-reorder -Wno-format-truncation -Wno-format)
