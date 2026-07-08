idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES
        TR_Compat
        TR_NVS
        driver
        esp_timer
        TR_RocketComputerTypes
        TR_I2C_Interface
        TR_I2S_Stream
        TR_LogToFlash
        TR_FlightLog
        TR_LoRa_Comms
        TR_RadioLink
        TR_Sensor_Data_Converter
        TR_Orientation
        TR_Coordinates
        TR_BLE_To_APP
        TR_INA230
        TR_OTA
        app_update
        esp_app_format
        nvs_flash
)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error -Wno-format -Wno-narrowing)

# Board revision selection (#411): `idf.py -DTR_BOARD_V8=1 build` compiles the
# V8 pin map (main/board/board_v8.h); default is V7. Use a separate build dir
# per variant (e.g. -B build_v8) — the flag is cached by CMake.
if(TR_BOARD_V8)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE TR_BOARD_V8=1)
endif()
