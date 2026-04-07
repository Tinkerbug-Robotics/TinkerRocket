idf_component_register(SRCS "TR_BLE_To_APP.cpp" INCLUDE_DIRS "." REQUIRES bt TR_Compat)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error -Wno-reorder -Wno-format-truncation -Wno-format)
