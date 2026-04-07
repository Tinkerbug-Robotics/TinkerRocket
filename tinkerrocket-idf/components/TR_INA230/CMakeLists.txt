idf_component_register(SRCS "TR_INA230.cpp" INCLUDE_DIRS "." REQUIRES TR_Compat PRIV_REQUIRES driver)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error -Wno-reorder -Wno-format-truncation -Wno-format)
