file(GLOB CRC_SRCS "*.cpp")
idf_component_register(SRCS ${CRC_SRCS} INCLUDE_DIRS "." REQUIRES TR_Compat)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error)
