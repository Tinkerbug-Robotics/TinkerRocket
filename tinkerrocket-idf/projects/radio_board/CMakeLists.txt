cmake_minimum_required(VERSION 3.16)
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../components")
# The modem is protocol-blind by design: no guidance, no BLE, no sensor stack.
set(EXCLUDE_COMPONENTS "TR_GuidancePN TR_GuidancePN_Stub TR_BLE_To_APP")

# Embed git short-sha + build timestamp into the firmware image header
# (esp_app_desc.version), same pattern as flight_computer / out_computer.
# Reported to the host in IDENTITY/BOOT (RadioModemProtocol.h).
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
set(PROJECT_VER "${TR_GIT_SHA}+${TR_BUILD_DATE}")


# Keep the GENERATED sdkconfig in the build dir, not at the project root.
#
# IDF's default is a single projects/{proj}/sdkconfig shared by every build
# dir, and it does NOT re-apply sdkconfig.defaults to an sdkconfig that already
# exists — nor does `idf.py fullclean` remove it, because it is not in the
# build dir. A root sdkconfig therefore outlives every clean and silently
# shadows this project's defaults forever.
#
# That is not hypothetical. It is why the fleet-wide boot NOR size was wrong in
# git for two days without anyone seeing it (2026-08-22 -> 2026-08-24): every
# root sdkconfig on the author's machine predated the change and still held the
# old value, so the bad default only ever appeared in a FRESH build dir — i.e.
# on a clean clone, or on CI, or the first time someone deleted build/. Moving
# the file here means a changed default takes effect when the build dir is
# recreated, which is where anyone would look for it.
set(SDKCONFIG "${CMAKE_BINARY_DIR}/sdkconfig")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(radio_board)
