idf_component_register(SRCS "TR_ISM6HG256.cpp" "ism6hg256x_reg.c" INCLUDE_DIRS "." REQUIRES TR_Compat)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error -Wno-reorder -Wno-format-truncation -Wno-format)
