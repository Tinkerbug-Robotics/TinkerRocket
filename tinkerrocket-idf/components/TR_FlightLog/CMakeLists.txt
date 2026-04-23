idf_component_register(
    SRCS "TR_FlightLog.cpp" "BlockStateBitmap.cpp" "FlightIndex.cpp" "WireFormat.cpp" "TR_NandBackend_esp.cpp" "NvsBitmapStore.cpp"
    INCLUDE_DIRS "include"
    REQUIRES TR_Compat CRC TR_LogToFlash TR_NVS
)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error -Wno-reorder -Wno-format-truncation -Wno-format)
