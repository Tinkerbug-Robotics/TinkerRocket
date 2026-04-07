idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES
        TR_Compat
        TR_NVS
        TR_RocketComputerTypes
        TR_Sensor_Data_Converter
        TR_Coordinates
        TR_BLE_To_APP
        TR_LoRa_Comms
        driver
        esp_timer
        fatfs
        sdmmc
        vfs
)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error -Wno-format)
