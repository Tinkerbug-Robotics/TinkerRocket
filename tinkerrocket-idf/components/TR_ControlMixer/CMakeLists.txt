idf_component_register(SRCS "TR_ControlMixer.cpp" INCLUDE_DIRS "." REQUIRES TR_Compat TR_PID)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error -Wno-reorder -Wno-format-truncation -Wno-format)
