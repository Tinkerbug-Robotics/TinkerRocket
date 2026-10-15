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
# #491 bench diagnostic: per-satellite C/N0 on the console, so a withheld
# position can be told from a lost signal. Off unless asked for; the flight
# path neither sets nor needs it.
if(TR_GNSS_COCOM_DIAG)
    add_compile_definitions(TR_GNSS_COCOM_DIAG=1)
    message(STATUS "TR_GNSS_COCOM_DIAG=1 -- per-satellite GNSS diagnostic ENABLED")
endif()
# #491 bench: the u-blox navigation rate, to test whether it changes what a boost
# costs. Unset, the flight's GNSS_UPDATE_RATE stands (18 Hz, config.h).
if(TR_GNSS_RATE_HZ)
    add_compile_definitions(TR_GNSS_RATE_HZ=${TR_GNSS_RATE_HZ})
    message(STATUS "TR_GNSS_RATE_HZ=${TR_GNSS_RATE_HZ} -- bench GNSS rate override")
endif()
# Bench diagnostic: watch what the I2S TX DMA really sends, frame by frame,
# with a loopback receiver on I2S1 (main/i2s_tx_monitor.h). ~80 KB of internal
# RAM and two ISR copies per DMA buffer; never set for flight.
if(TR_I2S_TX_MONITOR)
    add_compile_definitions(TR_I2S_TX_MONITOR=1)
    message(STATUS "TR_I2S_TX_MONITOR=1 -- I2S TX monitor ENABLED (bench)")
endif()
# #1600 phase 4: the raw-GNSS shadow filter runs wherever the LC86G driver does
# (main/config.h). -DTR_RAW_GNSS_SHADOW=0 builds an LC86G image without it, for
# a bench A/B of what it costs on core 0.
if(DEFINED TR_RAW_GNSS_SHADOW)
    add_compile_definitions(TR_RAW_GNSS_SHADOW=${TR_RAW_GNSS_SHADOW})
    message(STATUS "TR_RAW_GNSS_SHADOW=${TR_RAW_GNSS_SHADOW} -- raw-GNSS shadow filter set by hand")
endif()
# #1600: the raw-GNSS filter is the navigation filter wherever it runs
# (main/config.h); -DTR_RAW_GNSS_PRIMARY=0 flies GpsInsEKF beside it instead.
if(DEFINED TR_RAW_GNSS_PRIMARY)
    add_compile_definitions(TR_RAW_GNSS_PRIMARY=${TR_RAW_GNSS_PRIMARY})
    message(STATUS "TR_RAW_GNSS_PRIMARY=${TR_RAW_GNSS_PRIMARY} -- raw-GNSS navigation filter set by hand")
endif()
# The heading from thrust (main/config.h): -DTR_HEADING_FIT=0 off, 1 observe
# (fit and log), 2 act (hand the fit's heading to the navigation filter).
if(DEFINED TR_HEADING_FIT)
    add_compile_definitions(TR_HEADING_FIT=${TR_HEADING_FIT})
    message(STATUS "TR_HEADING_FIT=${TR_HEADING_FIT} -- the heading from thrust set by hand")
endif()
# Bench: what the heading fit costs core 0, timed once when the raw-GNSS task
# starts (TR_RawGnssShadow/HeadingFitBench.h). Never set for flight.
if(TR_HEADING_FIT_BENCH)
    add_compile_definitions(TR_HEADING_FIT_BENCH=1)
    message(STATUS "TR_HEADING_FIT_BENCH=1 -- the heading fit's bench ENABLED (bench)")
endif()
# Bench: the pad's heading turned this many degrees (added to the magnetic
# declination), so the heading fit has a known error to find -- a rig replay
# with and without it should differ by it in the fit's turn. Never set for flight.
if(DEFINED TR_BENCH_HEADING_ERROR_DEG)
    add_compile_definitions(TR_BENCH_HEADING_ERROR_DEG=${TR_BENCH_HEADING_ERROR_DEG})
    message(STATUS "TR_BENCH_HEADING_ERROR_DEG=${TR_BENCH_HEADING_ERROR_DEG} -- the pad heading turned on purpose (bench)")
endif()
# Bench (local, rig comparison): act's heading prior and hand-off threshold.
foreach(_v TR_BENCH_HEADING_PAD_SIGMA_DEG TR_BENCH_HEADING_HANDOFF_DEG)
    if(DEFINED ${_v})
        add_compile_definitions(${_v}=${${_v}})
        message(STATUS "${_v}=${${_v}} (bench)")
    endif()
endforeach()
# Bench (rig comparison): act's attitude gate from launch -- 1 opens it whole
# (RawGnssShadow heading_gate_full), 0 leaves the heading's -- and the tilt's
# floor at launch, deg (heading_launch_tilt_sigma_deg). Never set for flight.
foreach(_v TR_BENCH_HEADING_GATE_FULL TR_BENCH_LAUNCH_TILT_SIGMA_DEG)
    if(DEFINED ${_v})
        add_compile_definitions(${_v}=${${_v}})
        message(STATUS "${_v}=${${_v}} (bench)")
    endif()
endforeach()
# Bench (rig comparison): the carrier range rates -- 0 off, 1 observed (the
# LC86G profile's), 2 fused (RawGnssEKFParams cr_mode) -- and a rate a
# half-cycle step explains repaired (1) rather than dropped (0). Each sets the
# receiver's profile before it is applied (main.cpp startRawShadow). Never set
# for flight.
foreach(_v TR_BENCH_CARRIER_MODE TR_BENCH_HALF_CYCLE_REPAIR)
    if(DEFINED ${_v})
        add_compile_definitions(${_v}=${${_v}})
        message(STATUS "${_v}=${${_v}} (bench)")
    endif()
endforeach()
# Bench: a per-section cycle and Xtensa performance-counter profile of the
# flight task, printed every 5 s (main/loop_profile.h). Never set for flight.
if(TR_LOOP_PROFILE)
    add_compile_definitions(TR_LOOP_PROFILE=1)
    message(STATUS "TR_LOOP_PROFILE=1 -- flight-loop section profiler ENABLED (bench)")
endif()
# Bench: 1 = the EKF runs from internal RAM (main/bench_iram.lf),
# 2 = loop_fc() as well. Measures the flash-fetch cost. Never set for flight.
if(TR_BENCH_IRAM)
    add_compile_definitions(TR_BENCH_IRAM=${TR_BENCH_IRAM})
    message(STATUS "TR_BENCH_IRAM=${TR_BENCH_IRAM} -- hot code in internal RAM (bench)")
endif()
# #491 bench: cold-start the u-blox at every boot, so repeated runs of one file
# start from the same state. Never set for flight.
if(TR_GNSS_BENCH_COLD_START)
    add_compile_definitions(TR_GNSS_BENCH_COLD_START=1)
    message(STATUS "TR_GNSS_BENCH_COLD_START=1 -- u-blox cold start at every boot (bench)")
endif()

if(TR_BOARD_M1)
    set(TR_BOARD_SUFFIX "-m1")
elseif(TR_BOARD_V10)
    set(TR_BOARD_SUFFIX "-v10")
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

# #1125: the same board suffix reaches the compiler, so the OTA receiver can
# refuse an image built for a different board. CMake stays the single source of
# truth — a second copy in config.h would drift, and this value decides whether
# a firmware image is installed.
add_compile_definitions(TR_BOARD_OTA_SUFFIX="${TR_BOARD_SUFFIX}")

# Board revision has to reach sdkconfig, not just the compiler. M1
# (rocket-computer-mini) is the only flight computer that is NOT an ESP32-P4 —
# its FC is an ESP32-S3RH2 — so the overlay changes CONFIG_IDF_TARGET as well
# as turning on that part's in-package PSRAM.
#
# For V9 and V10 the overlay sets nothing any more. Until #916 it raised the boot
# NOR declaration to 16 MB for the GD25Q128 those boards carry; since the
# owner's 2026-09-09 decision every project declares the smallest standard size
# that fits its OWN partition table, not the part fitted — 8 MB here, from
# sdkconfig.defaults, on every board — and
# tinkerrocket-idf/tools/check_flash_declarations.py enforces it.
# Under-declaring only costs a V9 the top half of its part plus a harmless
# "Detected size(16384k) larger than ... (8192k)" boot line; over-declaring is
# the direction that boot-loops an 8 MB V7/V8 inside the bootloader, before
# any firmware runs. The comments-only overlay stays as the V9 per-board hook,
# so a future V9-only setting has the same home the M1 branch below uses.
#
# Both lines matter, and the second is the one that is easy to leave out.
# SDKCONFIG_DEFAULTS picks the overlay. SDKCONFIG moves the GENERATED config
# into the build dir, because IDF's default is a single sdkconfig at the
# PROJECT ROOT shared by every build dir, and it does not re-apply defaults to
# an sdkconfig that already exists. Without it, builds with different -B dirs
# quietly share one config — a wrong target or a wrong flash size ride into
# the next build with a correct-looking log. `idf.py -B <dir> reconfigure`
# re-seeds from these defaults; a root sdkconfig is now ignored (untracked).
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
    # The mini's magnetometer (U3) is a QST QMC5883P, not the IIS2MDC every
    # other board carries (#797).  This swaps the TR_QMC5883P driver into
    # TR_Sensor_Collector's IIS2MDC-named slot and stamps the log's mag_type
    # accordingly (#1312).  Project-wide, like TR_GNSS_DRIVER_LC86 on the
    # mini, because the seam lives in a component; config.h refuses an M1
    # build that lacks it.
    add_compile_definitions(TR_MAG_DRIVER_QMC5883P=1)
    message(STATUS "flight_computer: magnetometer driver QMC5883P (TR_MAG_DRIVER_QMC5883P=1)")
    # The mini's GNSS (U5) is a Quectel LC86G on THIS processor's UART, not the
    # u-blox every other board carries.  The same kind of seam as the
    # magnetometer above, and it had been missed: only the single-MCU
    # rocket_computer_mini project set it, so the M1 flight-computer image —
    # the one that actually flies on the fabbed dual-MCU board — was speaking
    # UBX at a module that answers NMEA (#1032).  config.h refuses an M1 build
    # that lacks it.
    add_compile_definitions(TR_GNSS_DRIVER_LC86=1)
    message(STATUS "flight_computer: GNSS driver LC86G (TR_GNSS_DRIVER_LC86=1)")
elseif(TR_BOARD_V10)
    # The V9 overlay is the Tinker-Mantis per-board hook, and it sets nothing
    # since #916; V10 shares it, so a setting both revisions need has one home.
    set(SDKCONFIG_DEFAULTS "sdkconfig.defaults;sdkconfig.defaults.v9")
    message(STATUS "flight_computer: sdkconfig overlay sdkconfig.defaults.v9 (shared with V9, comments only — boot NOR keeps the base 8 MB declaration, #916)")
    # U3 on the V10 is a QST QMC5883P (0x2C, no DRDY), where the V9 fits an ST
    # IIS2MDC. The same seam the Beetle uses; config.h refuses a V10 build
    # without it, and the board rotation is board_v10.h's
    # MAG_ROT_Z_DEG_QMC5883P (#1590, #1593).
    add_compile_definitions(TR_MAG_DRIVER_QMC5883P=1)
    message(STATUS "flight_computer: magnetometer driver QMC5883P (TR_MAG_DRIVER_QMC5883P=1)")
elseif(TR_BOARD_V9)
    set(SDKCONFIG_DEFAULTS "sdkconfig.defaults;sdkconfig.defaults.v9")
    # V9 fits an ST IIS2MDC. This image asks for either chip at boot (#1590),
    # which it needed while V10's QMC5883P shared the V9 header; V10 has its
    # own build above since #1593.
    add_compile_definitions(TR_MAG_DRIVER_AUTO=1)
    message(STATUS "flight_computer: magnetometer driver auto (IIS2MDC or QMC5883P, TR_MAG_DRIVER_AUTO=1)")
    message(STATUS "flight_computer: sdkconfig overlay sdkconfig.defaults.v9 (comments only since #916 — boot NOR keeps the base 8 MB declaration, sized to the partition table)")
endif()
set(SDKCONFIG "${CMAKE_BINARY_DIR}/sdkconfig")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(flight_computer)
