idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES
        TR_Compat
        TR_NVS
        TR_RocketComputerTypes
        TR_Sensor_Collector
        TR_Sensor_Collector_Sim
        TR_I2C_Interface
        TR_I2S_Stream
        TR_Sensor_Data_Converter
        TR_GpsInsEKF
        TR_KinematicChecks
        TR_ServoControl_ledc_mult
        TR_GuidancePN
        TR_ControlMixer
        TR_GNSSReceiverUBlox_Serial
        TR_IMU_Int_V2
        driver
        esp_timer
        nvs_flash
)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error -Wno-format)
