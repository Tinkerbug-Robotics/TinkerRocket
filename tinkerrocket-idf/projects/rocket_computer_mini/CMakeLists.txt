cmake_minimum_required(VERSION 3.16)
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../components")
# The mini has no servos, so no control surfaces, so no guidance — exclude both
# the real submodule and the stub so neither ends up in the component list
# (same reasoning as out_computer).
set(EXCLUDE_COMPONENTS "TR_GuidancePN TR_GuidancePN_Stub")

# The mini's receiver is a Quectel LC86G, not a u-blox: swap the GNSS driver
# type inside TR_Sensor_Collector (see the seam note in its header). Project-
# wide definition, same mechanism as flight_computer's TR_GUIDANCE_AVAILABLE.
add_compile_definitions(TR_GNSS_DRIVER_LC86=1)

# Embed git short-sha + build timestamp into the firmware image header
# (esp_app_desc.version), max 32 chars. Read at runtime via
# esp_app_get_description()->version and surfaced in BLE config_identity's
# "fw" field. See docs/plans/08-ota-firmware-update.md §3.
execute_process(
    COMMAND git rev-parse --short HEAD
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE TR_GIT_SHA
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)
if(NOT TR_GIT_SHA)
    set(TR_GIT_SHA "unknown")
endif()
execute_process(
    COMMAND git diff-index --quiet HEAD --
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    RESULT_VARIABLE TR_GIT_DIRTY_CODE
    OUTPUT_QUIET ERROR_QUIET
)
if(NOT TR_GIT_DIRTY_CODE EQUAL 0)
    set(TR_GIT_SHA "${TR_GIT_SHA}-dirty")
endif()
string(TIMESTAMP TR_BUILD_DATE "%Y%m%d-%H%M" UTC)
# Board revision marker: the board comes ONLY from -DTR_MINI_BOARD=<n>
# (default 1; see main/config.h). Non-default revisions are stamped into the
# version string so a mismatched image is identifiable from the boot log, BLE
# config_identity, and OTA metadata. Suffix before the timestamp so it
# survives the 32-byte clip (same reasoning as out_computer's -v8 marker).
if(TR_MINI_BOARD AND NOT TR_MINI_BOARD EQUAL 1)
    set(TR_BOARD_SUFFIX "-b${TR_MINI_BOARD}")
else()
    set(TR_BOARD_SUFFIX "")
endif()
set(PROJECT_VER "${TR_GIT_SHA}${TR_BOARD_SUFFIX}+${TR_BUILD_DATE}")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(rocket_computer_mini)
