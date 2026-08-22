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
# NB: main/CMakeLists.txt runs the same EXISTS check to decide whether to
# list TR_GuidancePN or TR_GuidancePN_Stub in its REQUIRES. Kept in sync
# by hand — both checks must agree.

# Exclude BLE and LoRa components not used by FlightComputer
list(APPEND EXCLUDE_COMPONENTS TR_BLE_To_APP TR_LoRa_Comms TR_INA230)

# Embed git short-sha + build timestamp into the firmware image header
# (esp_app_desc.version), max 32 chars. Read at runtime via
# esp_app_get_description()->version and reported as the "fw" field so the OTA
# flow can verify the running FC image — the OC relays it up over BLE (#8 §7).
# (Separate from the FW_GIT_SHA compile-def in main/CMakeLists.txt, which feeds
# the per-flight settings snapshot.)
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
# Board revision marker (#411): every build is identifiable from the app
# version string (BLE config_identity "fw" field, boot log, OTA metadata), so
# an image flashed onto the wrong revision can be spotted after the fact.
# main/CMakeLists.txt is what validates the flag; this only labels the image.
if(TR_BOARD_M1)
    set(TR_BOARD_SUFFIX "-m1")
elseif(TR_BOARD_V9)
    set(TR_BOARD_SUFFIX "-v9")
elseif(TR_BOARD_V8)
    set(TR_BOARD_SUFFIX "-v8")
elseif(TR_BOARD_V7)
    set(TR_BOARD_SUFFIX "-v7")
else()
    set(TR_BOARD_SUFFIX "")
endif()
set(PROJECT_VER "${TR_GIT_SHA}${TR_BOARD_SUFFIX}+${TR_BUILD_DATE}")

# Board revision has to reach sdkconfig, not just the compiler. M1
# (rocket-computer-mini) is the only flight computer that is NOT an ESP32-P4 —
# its FC is an ESP32-S3RH2 — so the overlay changes CONFIG_IDF_TARGET as well
# as turning on that part's in-package PSRAM.
#
# Both lines matter. SDKCONFIG_DEFAULTS picks the overlay; SDKCONFIG moves the
# GENERATED config into the build dir, because the default is one shared
# sdkconfig at the project root and IDF does not re-apply defaults to an
# existing one. Without the second line a root sdkconfig written by an M1 build
# would carry esp32s3 into the next P4 build, and a target mismatch surfaces as
# a wall of unrelated compile errors rather than a clear message.
if(TR_BOARD_M1)
    set(SDKCONFIG_DEFAULTS "sdkconfig.defaults;sdkconfig.defaults.m1")
    # CONFIG_IDF_TARGET in the overlay is NOT enough on its own. idf.py
    # resolves the target before SDKCONFIG_DEFAULTS is applied, so an M1 build
    # with only the overlay produces a perfectly successful ESP32-P4 image
    # carrying the ESP32-S3 pin map — the exact silent wrong-chip build this
    # project refuses to allow for pin maps. Force it into the cache here, and
    # main/CMakeLists.txt asserts the result after resolution.
    set(IDF_TARGET "esp32s3" CACHE STRING "M1 flight computer is an ESP32-S3" FORCE)
    message(STATUS "flight_computer: sdkconfig overlay sdkconfig.defaults.m1 (ESP32-S3 target, PSRAM on)")
endif()
set(SDKCONFIG "${CMAKE_BINARY_DIR}/sdkconfig")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(flight_computer)
