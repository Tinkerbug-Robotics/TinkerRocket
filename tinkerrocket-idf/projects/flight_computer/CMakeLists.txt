cmake_minimum_required(VERSION 3.16)
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../components")

# TR_GuidancePN is a private submodule. Detect whether it's initialized
# (working tree populated) and either include it + the stub is excluded,
# or exclude it + the stub fills in so consumer code compiles unchanged.
set(_guidance_real_src "${CMAKE_CURRENT_LIST_DIR}/../../components/TR_GuidancePN/TR_GuidancePN.cpp")
if(EXISTS "${_guidance_real_src}")
    message(STATUS "TR_GuidancePN submodule found — guidance compiled in")
    list(APPEND EXCLUDE_COMPONENTS TR_GuidancePN_Stub)
    add_compile_definitions(TR_GUIDANCE_AVAILABLE=1)
else()
    message(STATUS "TR_GuidancePN submodule not initialized — building with no-op stub")
    list(APPEND EXCLUDE_COMPONENTS TR_GuidancePN)
    add_compile_definitions(TR_GUIDANCE_AVAILABLE=0)
endif()

# Exclude BLE and LoRa components not used by FlightComputer
list(APPEND EXCLUDE_COMPONENTS TR_BLE_To_APP TR_LoRa_Comms TR_INA230)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(flight_computer)
