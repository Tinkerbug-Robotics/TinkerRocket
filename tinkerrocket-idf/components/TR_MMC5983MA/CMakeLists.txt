idf_component_register(SRCS "TR_MMC5983MA.cpp" INCLUDE_DIRS "." REQUIRES TR_Compat)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error -Wno-reorder -Wno-format-truncation -Wno-format)
