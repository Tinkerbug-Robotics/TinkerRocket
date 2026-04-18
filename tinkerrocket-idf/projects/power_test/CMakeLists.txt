cmake_minimum_required(VERSION 3.16)
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../components")
set(EXCLUDE_COMPONENTS
    TR_I2C_Interface TR_I2S_Stream TR_LogToFlash
    TR_LoRa_Comms TR_ISM6HG256 TR_BMP585 TR_MMC5983MA
    TR_GNSSReceiverUBlox_Serial TR_GpsInsEKF TR_KinematicChecks
    TR_Coordinates TR_ControlMixer TR_GuidancePN TR_PID
    TR_Sensor_Collector TR_Sensor_Collector_Sim TR_IMU_Int_V2
    RadioLib)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(power_test)
