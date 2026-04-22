idf_component_register(
    SRCS "TR_FlightLog.cpp" "BlockStateBitmap.cpp" "FlightIndex.cpp" "WireFormat.cpp" "TR_NandBackend_esp.cpp"
    INCLUDE_DIRS "include"
    REQUIRES TR_Compat CRC
)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error -Wno-reorder -Wno-format-truncation -Wno-format)
