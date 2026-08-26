cmake_minimum_required(VERSION 3.16)
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../components")
# base_station doesn't use guidance — exclude both the real submodule and
# the stub so neither ends up in the component list.
set(EXCLUDE_COMPONENTS "TR_GuidancePN TR_GuidancePN_Stub")

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
set(PROJECT_VER "${TR_GIT_SHA}+${TR_BUILD_DATE}")

# Board revision has to reach sdkconfig, not just the compiler — the pattern
# out_computer/CMakeLists.txt records under #822, needed here for the boot NOR.
# V3 (PCB V5/V6) fits a 16 MB GD25Q128ESIG where V1/V2 carry an 8 MB W25Q64,
# and a 16 MB declaration on an 8 MB part is a boot loop inside the bootloader,
# before any firmware runs. The overlay also switches the partition table,
# because V3's spiffs is 9.94 MB and does not fit an 8 MB layout at all — see
# sdkconfig.defaults.v3.
#
# Both lines matter, and the second is the one that is easy to leave out.
# SDKCONFIG_DEFAULTS picks the overlay. SDKCONFIG moves the GENERATED config
# into the build dir, because IDF's default is a single sdkconfig at the
# PROJECT ROOT shared by every build dir, and it does not re-apply defaults to
# an sdkconfig that already exists. Without it, `-B build_v3` and `-B build_v2`
# quietly share one config and whichever ran first decides the flash size and
# partition table for both.
#
# TR_BS_BOARD stays optional (main/CMakeLists.txt warns and defaults to V2) and
# the default now lands on the SAFE value: an 8 MB image boots on a 16 MB
# board, so a forgotten flag costs a V3 the top half of its part rather than
# costing a V2 its ability to boot.
#
# One-time effect on existing checkouts: the root projects/base_station/
# sdkconfig is now ignored (it is untracked, so nothing is lost from git).
# `idf.py -B <dir> reconfigure` re-seeds from these defaults.
if(TR_BS_BOARD EQUAL 3)
    set(SDKCONFIG_DEFAULTS "sdkconfig.defaults;sdkconfig.defaults.v3")
    message(STATUS "base_station: sdkconfig overlay sdkconfig.defaults.v3 (16 MB boot NOR, partitions_v3.csv)")
endif()
set(SDKCONFIG "${CMAKE_BINARY_DIR}/sdkconfig")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(base_station)
