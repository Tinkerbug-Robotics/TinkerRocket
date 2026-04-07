idf_component_register(SRCS "TR_OUT_Page.cpp" INCLUDE_DIRS "." REQUIRES TR_Compat TR_RocketComputerTypes)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error -Wno-reorder -Wno-format-truncation -Wno-format)
