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
        TR_LoRa_Comms
        TR_Sensor_Data_Converter
        TR_Coordinates
        TR_BLE_To_APP
        TR_INA230
        nvs_flash
)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error -Wno-format -Wno-narrowing)
