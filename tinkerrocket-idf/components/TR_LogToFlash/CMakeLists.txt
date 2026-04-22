idf_component_register(SRCS "TR_LogToFlash.cpp" "lfs.c" "lfs_util.c" INCLUDE_DIRS "." REQUIRES TR_Compat TR_RocketComputerTypes CRC TR_NVS PRIV_REQUIRES freertos heap esp_system)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error -Wno-reorder -Wno-format-truncation -Wno-format)
