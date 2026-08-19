cmake_minimum_required(VERSION 3.16)
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../components")
# out_computer doesn't use guidance — exclude both the real submodule and
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
# Board revision marker (#411): V8/V9 builds are identifiable from the app
# version string (BLE config_identity "fw" field, boot log, OTA metadata), so
# an image can be traced back to the board it was built for. Since #822 that
# matters at runtime and not only for provenance: the two select different
# headers (V9/V10 has no MRAM), so a "-v8" image on V9 hardware is a real
# misconfiguration, not just a mislabel.
if(TR_BOARD_V9)
    set(TR_BOARD_SUFFIX "-v9")
elseif(TR_BOARD_V8)
    set(TR_BOARD_SUFFIX "-v8")
else()
    set(TR_BOARD_SUFFIX "")
endif()
# Bench-debug marker: a TR_OC_BENCH_DEBUG image keeps light sleep off for the
# whole session (see main/CMakeLists.txt), so it draws ~22 mA instead of ~1 mA.
# Stamping it here makes that visible everywhere the version travels — boot
# log, BLE config_identity "fw", the app's Firmware screen, OTA metadata — so
# one can't end up on a rocket unnoticed.
#
# esp_app_desc.version is 32 bytes, and "<sha>-dirty-bench+<date>" overflows it
# — the suffixes come BEFORE the timestamp deliberately, so the safety marker
# always survives and only the build date gets clipped.
if(TR_OC_BENCH_DEBUG)
    set(TR_BENCH_SUFFIX "-bench")
else()
    set(TR_BENCH_SUFFIX "")
endif()
set(PROJECT_VER "${TR_GIT_SHA}${TR_BOARD_SUFFIX}${TR_BENCH_SUFFIX}+${TR_BUILD_DATE}")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(out_computer)
