cmake_minimum_required(VERSION 3.16)
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../components")
# Exclude BLE and LoRa components not used by FlightComputer
set(EXCLUDE_COMPONENTS "TR_BLE_To_APP TR_LoRa_Comms TR_OUT_Page TR_INA230")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(flight_computer)
