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
# Bench only: make setup_fc() fail at the sensor step, to exercise the setup
# watchdog and rescue mode (main/fc_rescue_policy.h) on a board. 1 = park there
# forever (a stall: restarts into rescue mode after the setup watchdog), 2 =
# abort() there (a crash loop: rescue mode after three boots). Flash it over
# USB: an image delivered over the air rolls back on its first restart, which
# tests the rollback instead. Never set for flight.
if(TR_TEST_SETUP_FAIL)
    add_compile_definitions(TR_TEST_SETUP_FAIL=${TR_TEST_SETUP_FAIL})
    message(WARNING "TR_TEST_SETUP_FAIL=${TR_TEST_SETUP_FAIL} -- setup_fc() fails ON PURPOSE (bench rescue-mode test image)")
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
# (fit and log), 2 act (GNSS turns the attitude from launch; the default where
# the raw filter navigates).
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
# Bench (local, rig comparison): act's heading prior -- with the
# magnetometer's heading (TR_BENCH_HEADING_PAD_SIGMA_DEG) and without it
# (TR_BENCH_HEADING_PAD_SIGMA_FALLBACK_DEG) -- and hand-off threshold.
foreach(_v TR_BENCH_HEADING_PAD_SIGMA_DEG TR_BENCH_HEADING_PAD_SIGMA_FALLBACK_DEG TR_BENCH_HEADING_HANDOFF_DEG)
    if(DEFINED ${_v})
        add_compile_definitions(${_v}=${${_v}})
        message(STATUS "${_v}=${${_v}} (bench)")
    endif()
endforeach()
# Bench (rig comparison): act's attitude gate in flight (RawGnssShadow
# heading_gate) -- 0 the heading's, 1 whole from launch, 2 the heading's until
# the flight computer's apogee call and whole from it -- and the tilt's floor
# at launch, deg (heading_launch_tilt_sigma_deg). Never set for flight.
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
# Bench (the mosaic-only image's CPU ladder, P1): the raw-GNSS filter's fused
# grid in ms of GPS time (TR_BENCH_RAW_FUSE_MS: 100 is the profile's 10 Hz,
# 200 its 5 Hz), the satellites it builds an epoch at most
# (TR_BENCH_RAW_SAT_CAP: 16 is the profile's), and the boost screen's
# threshold in g (TR_BENCH_BOOST_SCREEN_G: 0 screens every epoch, its worst
# cost), and the Doppler change check (TR_BENCH_DV_CHECK: 1 observes, 2 acts,
# at Grid B's recommended settings; exp/v9-dvcheck). Each sets the filter's
# start (main.cpp startRawShadow). Never set for flight.
#
# These and TR_RAW_GNSS_IRAM below act only where the raw-GNSS filter
# navigates on the mosaic-G5 (main.cpp's TR_RAW_NAV_MOSAIC): any other build
# refuses them rather than build an image that silently ignores them.
if(TR_GNSS_MOSAIC AND (NOT DEFINED TR_RAW_GNSS_SHADOW OR TR_RAW_GNSS_SHADOW))
    set(_tr_mosaic_rawnav ON)
else()
    set(_tr_mosaic_rawnav OFF)
endif()
set(_tr_mosaic_rawnav_hooks TR_BENCH_RAW_START_FAIL TR_BENCH_RESTORE_SIM TR_BENCH_PANIC_AT_S
    TR_BENCH_PANIC_FILTER_AT_S TR_BENCH_RAW_STOP_AT_S)
foreach(_v TR_BENCH_RAW_FUSE_MS TR_BENCH_RAW_SAT_CAP TR_BENCH_BOOST_SCREEN_G TR_BENCH_DV_CHECK TR_RAW_GNSS_IRAM
        ${_tr_mosaic_rawnav_hooks})
    if(DEFINED ${_v} AND NOT _tr_mosaic_rawnav)
        message(FATAL_ERROR "${_v} is for the mosaic-only image's raw-GNSS filter (-DTR_GNSS_MOSAIC=1 on a V8 or V9): this build would ignore it. A build dir keeps every -D value: use a fresh one.")
    endif()
endforeach()
foreach(_v TR_BENCH_RAW_FUSE_MS TR_BENCH_RAW_SAT_CAP TR_BENCH_BOOST_SCREEN_G TR_BENCH_DV_CHECK)
    if(DEFINED ${_v})
        add_compile_definitions(${_v}=${${_v}})
        message(STATUS "${_v}=${${_v}} (bench)")
    endif()
endforeach()
# Bench hooks for the mosaic-only image's robustness bench (P3, gate G8), one
# image each, never flown (main.cpp, TR_RAW_NAV_MOSAIC):
#   TR_BENCH_RAW_START_FAIL=1     the filter's start fails before it allocates:
#                                 NO NAVIGATION FILTER and FCB_DEG_SENSORS
#   TR_BENCH_RESTORE_SIM=1        a sim flight's snapshot is restored like a
#                                 real one (power cut mid sim), every pyro dry
#   TR_BENCH_PANIC_AT_S=N         one panic N s after the loop starts, once a
#                                 power-on: the crash boot after it
#   TR_BENCH_PANIC_FILTER_AT_S=N  the filter's task aborts N s after the filter
#                                 runs in flight, every boot: with
#                                 TR_BENCH_RESTORE_SIM, the crash loop
#   TR_BENCH_RAW_STOP_AT_S=N      the filter stops N s into each flight: held
#                                 over and restarted from its last good state
foreach(_v ${_tr_mosaic_rawnav_hooks})
    if(DEFINED ${_v})
        add_compile_definitions(${_v}=${${_v}})
        message(WARNING "${_v}=${${_v}} -- a BENCH hook of the mosaic-only image (P3): never fly this image")
    endif()
endforeach()
# Bench: the raw-GNSS filter's EKF and measurement builder in internal RAM
# (main/raw_gnss_iram.lf, the Beetle's placement) on another board too:
# -DTR_RAW_GNSS_IRAM=1 on the mosaic-only image, to measure what the P4's
# PSRAM-and-flash fetches cost it. main/CMakeLists.txt places the fragment.
if(DEFINED TR_RAW_GNSS_IRAM)
    add_compile_definitions(TR_RAW_GNSS_IRAM=${TR_RAW_GNSS_IRAM})
    message(STATUS "TR_RAW_GNSS_IRAM=${TR_RAW_GNSS_IRAM} -- the raw-GNSS filter in internal RAM (bench)")
endif()
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
# RF rig bench switches for the mosaic-G5 (never set for flight): a C/N0 and
# tracking line every 5 s, a cold start (erst,Soft,PVTData+SatData) after
# the setup at each boot, so a simulated sky is not searched with the real
# sky's time and ephemerides, and a time-tag trace line every whole second
# (TOW, RxClkBias, CumClkJumps, the bias the tag carried, the 1PPS edge, the
# PPS check's residual and whether the epoch went out timed: the raw outputs'
# MosaicTimeTag.h, so it needs the raw-GNSS filter's image).
#
# The rig replay (#1600: -DTR_SIM_REPLAY_BLOB=FILE, TR_Sensor_Collector_Sim/
# CMakeLists.txt) plays a flight's sensors against the receiver's GPS time,
# which only two receivers give it: the M1's LC86G (its timed fixes) and the
# mosaic-only image's mosaic-G5 (the raw-GNSS filter's timed epochs, which its
# GNSS task drains). Any other build would play the pad for ever, so it is
# refused. On the mosaic-only image a replay build
#   - cold-starts the receiver at each boot, as TR_BENCH_MOSAIC_COLD_START
#     does (the LC86G's replay build does its own,
#     TR_GNSSReceiverLC86_Serial/CMakeLists.txt): the rig's sky is another
#     day and place than the one the receiver last tracked, and every good rig
#     capture of the mosaic was cold-started first;
#   - tells main.cpp it flies the rig's sky (TR_RAW_NAV_RIG_REPLAY): the
#     sky's broadcast ionosphere wins over the app's or NVS's, whatever their
#     days, and no set dated more than a day from the sky's is taken
#     (raw_iono_rule.h offerRigReplay); the console carries the navigation
#     state at its own instant and each apogee voter's changes, which the
#     rig's scorer reads (tools/rig_replay/score_v9.py); and the replayed
#     high-g counts carry the converter's high-g bias, so where the low-g rail
#     hands over the flight's own force comes out (SimHighGBias.h; the low-g
#     hand offset is off in any sim, #1629).
# The rig's files are too short to broadcast GPSIon, so the build should carry
# the sky's own set: -DTR_SIM_REPLAY_GPS_IONO=$(tools/rig_replay/
# rinex_gps_iono.py <the scenario's RINEX>) (TR_Sensor_Collector_Sim). Without
# it the filter builds the rig's sky with no Klobuchar.
if(TR_SIM_REPLAY_BLOB)
    if(TR_GNSS_MOSAIC AND NOT _tr_mosaic_rawnav)
        message(FATAL_ERROR "TR_SIM_REPLAY_BLOB on the mosaic-only image takes its clock from the raw-GNSS filter's timed epochs: build it with the filter (no -DTR_RAW_GNSS_SHADOW=0)")
    elseif(NOT TR_GNSS_MOSAIC AND NOT TR_BOARD_M1)
        message(FATAL_ERROR "TR_SIM_REPLAY_BLOB needs a receiver whose time sets the replay's clock: the M1's LC86G (-DTR_BOARD_M1=1) or the mosaic-only image (-DTR_GNSS_MOSAIC=1 on a V8 or V9)")
    endif()
    if(TR_GNSS_MOSAIC)
        set(TR_BENCH_MOSAIC_COLD_START ON)
        add_compile_definitions(TR_RAW_NAV_RIG_REPLAY=1)
        message(STATUS "flight_computer: RIG REPLAY of the mosaic-only image (TR_RAW_NAV_RIG_REPLAY=1): the receiver cold-starts at boot, the sky's broadcast ionosphere wins")
        if(NOT TR_SIM_REPLAY_GPS_IONO)
            message(WARNING "flight_computer: RIG REPLAY without the sky's ionosphere (-DTR_SIM_REPLAY_GPS_IONO): the rig's files never broadcast GPSIon, so the filter builds the sky with no Klobuchar (a few metres in the pseudoranges)")
        endif()
    endif()
endif()
if(TR_SIM_REPLAY_GPS_IONO AND NOT (TR_SIM_REPLAY_BLOB AND TR_GNSS_MOSAIC))
    message(FATAL_ERROR "TR_SIM_REPLAY_GPS_IONO is the rig sky's ionosphere for a rig replay (-DTR_SIM_REPLAY_BLOB) of the mosaic-only image (-DTR_GNSS_MOSAIC=1)")
endif()
foreach(_v TR_BENCH_MOSAIC_CN0 TR_BENCH_MOSAIC_COLD_START TR_BENCH_MOSAIC_TAG_TRACE)
    if(${_v} AND NOT TR_GNSS_MOSAIC)
        message(FATAL_ERROR "${_v} is for the mosaic-only image (-DTR_GNSS_MOSAIC=1 on a V8 or V9)")
    endif()
endforeach()
if(TR_BENCH_MOSAIC_TAG_TRACE AND NOT _tr_mosaic_rawnav)
    message(FATAL_ERROR "TR_BENCH_MOSAIC_TAG_TRACE traces the raw-GNSS filter's time tag: this build has no raw outputs (TR_RAW_GNSS_SHADOW=0)")
endif()
if(TR_BENCH_MOSAIC_TAG_TRACE)
    add_compile_definitions(TR_BENCH_MOSAIC_TAG_TRACE=1)
    message(WARNING "TR_BENCH_MOSAIC_TAG_TRACE=1 -- mosaic time-tag trace line every whole second (bench)")
endif()
if(TR_BENCH_MOSAIC_CN0)
    add_compile_definitions(TR_BENCH_MOSAIC_CN0=1)
    message(WARNING "TR_BENCH_MOSAIC_CN0=1 -- mosaic tracking/C/N0 console line (bench)")
endif()
if(TR_BENCH_MOSAIC_COLD_START)
    add_compile_definitions(TR_BENCH_MOSAIC_COLD_START=1)
    message(WARNING "TR_BENCH_MOSAIC_COLD_START=1 -- mosaic-G5 cold start at every boot (bench)")
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
# Bench marker, as the out_computer stamps its own: a build with any bench
# switch above -- every TR_BENCH_* the cache holds, whatever its value, and
# the others as each is applied -- says "-bench" after the board in its
# version, which the boot log, the app's Firmware screen and OTA show. A bench
# image installed over the air is then never taken for the flight image it
# was built beside, and a bench build dir (its cache keeps every -D) reused
# for a flight build stays marked. Before the date, so esp_app_desc's 31
# characters clip the date and never the marker.
set(TR_BENCH_SUFFIX "")
get_cmake_property(_tr_cache_vars CACHE_VARIABLES)
foreach(_v ${_tr_cache_vars})
    if(_v MATCHES "^TR_BENCH_")
        set(TR_BENCH_SUFFIX "-bench")
    endif()
endforeach()
# A rig replay image (TR_SIM_REPLAY_BLOB) plays a canned flight from boot as a
# sim, every pyro dry: the last image to take for the flight one.
if(DEFINED TR_RAW_GNSS_IRAM OR TR_LOOP_PROFILE OR TR_HEADING_FIT_BENCH OR TR_I2S_TX_MONITOR OR TR_TEST_SETUP_FAIL
   OR TR_GNSS_BENCH_COLD_START OR TR_GNSS_COCOM_DIAG OR TR_GNSS_RATE_HZ OR TR_SIM_REPLAY_BLOB)
    set(TR_BENCH_SUFFIX "-bench")
endif()
if(TR_BENCH_SUFFIX)
    message(STATUS "flight_computer: a BENCH image (version marked ${TR_BENCH_SUFFIX}): never fly it")
endif()
set(PROJECT_VER "${TR_GIT_SHA}${TR_BOARD_SUFFIX}${TR_BENCH_SUFFIX}+${TR_BUILD_DATE}")

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
# A V9 or a V8 flying a Septentrio mosaic-G5 on J1 instead of its u-blox
# carrier (`idf.py -B build_v9_mosaic -DTR_BOARD_V9=1 -DTR_GNSS_MOSAIC=1 build`,
# or `-B build_v8_rawnav -DTR_BOARD_V8=1 -DTR_GNSS_MOSAIC=1`: the Goblin's
# V8). The same seam as TR_GNSS_DRIVER_LC86 on the M1: the driver is the
# build's choice, and config.h refuses it on any other board. The V8's J1 is
# the V9's (the receiver's UART on GPIO3/GPIO4, its 1PPS on pin 5 = GPIO2), so
# the harness lands the same way on both. Its own overlay puts the UART
# interrupt in IRAM: at 2 Mbaud the RX FIFO lasts 0.64 ms, shorter than a
# flash write keeps a cache-dependent interrupt waiting.
#
# sdkconfig.defaults.rawnav turns the in-package PSRAM on, for the raw-GNSS
# navigation filter this image runs (#1600 on the mosaic): it does not fit in
# internal RAM. Only this image takes it -- the V9 and V8 auto images also take
# sdkconfig.defaults.mosaic, so PSRAM cannot go there -- and
# -DTR_RAW_GNSS_SHADOW=0 builds the mosaic image without the filter and
# without PSRAM, as every board flew before (sdkconfig.defaults, #851).
# main/CMakeLists.txt checks the resolved sdkconfig against this rule: a build
# dir's sdkconfig keeps CONFIG_SPIRAM=y once this overlay has written it.
if(TR_GNSS_MOSAIC)
    if(NOT TR_BOARD_V9 AND NOT TR_BOARD_V8)
        message(FATAL_ERROR "TR_GNSS_MOSAIC is for the V9 (-DTR_BOARD_V9=1) or the V8 (-DTR_BOARD_V8=1): their J1 is where the harness lands")
    endif()
    if(TR_GNSS_AUTO)
        message(FATAL_ERROR "TR_GNSS_MOSAIC and TR_GNSS_AUTO are both on: the mosaic-only image or the image that finds either receiver, not both")
    endif()
    add_compile_definitions(TR_GNSS_DRIVER_MOSAIC=1)
    # The V8 (legacy) sets no defaults list of its own: start it from the base
    # file, as the auto image below does, or the overlays alone would replace
    # it and the build lose its P4 target.
    if(NOT SDKCONFIG_DEFAULTS)
        set(SDKCONFIG_DEFAULTS "sdkconfig.defaults")
    endif()
    list(APPEND SDKCONFIG_DEFAULTS "sdkconfig.defaults.mosaic")
    if(NOT DEFINED TR_RAW_GNSS_SHADOW OR TR_RAW_GNSS_SHADOW)
        list(APPEND SDKCONFIG_DEFAULTS "sdkconfig.defaults.rawnav")
        # The receiver sends every GPS, Galileo and BeiDou ephemeris it
        # tracks, ~50 satellites over a 2-4 h pad wait: the raw-GNSS filter's
        # store holds 128 (the Beetle's 64). Project-wide, before project(),
        # so TR_GNSSRaw, TR_RawGnssShadow and main lay the pool out alike;
        # main.cpp checks at boot that they did (ephCapacityAgrees).
        add_compile_definitions(TR_EPH_CAPACITY=128)
        message(STATUS "flight_computer: GNSS driver Septentrio mosaic-G5 (TR_GNSS_DRIVER_MOSAIC=1), overlays sdkconfig.defaults.mosaic + sdkconfig.defaults.rawnav (PSRAM on)")
        message(STATUS "flight_computer: raw-GNSS NAVIGATION filter; GpsInsEKF not instantiated (ephemeris store 128)")
    else()
        message(STATUS "flight_computer: GNSS driver Septentrio mosaic-G5 (TR_GNSS_DRIVER_MOSAIC=1), overlay sdkconfig.defaults.mosaic (TR_RAW_GNSS_SHADOW=0: no raw-GNSS filter, PSRAM off)")
    endif()
endif()
# The V9 image finds its GNSS receiver at boot: the stock u-blox SAM-M10Q
# carrier or the mosaic-G5 harness, both on J1 (TR_Sensor_Collector's
# TR_GNSS_DRIVER_AUTO, gnss_auto_driver.h). On for the V9 unless
# -DTR_GNSS_AUTO=0, which builds the u-blox-only image every V9 flew before;
# -DTR_GNSS_MOSAIC=1 builds the mosaic-only one. It takes the mosaic's overlay
# too, because it may find the mosaic: at 2 Mbaud the UART interrupt has to
# run through flash writes (gnss_auto_driver.cpp refuses a build dir whose
# sdkconfig predates it). -DTR_BOARD_V8=1 -DTR_GNSS_AUTO=1 builds it for a V8
# as well, on request: the V8's J1 is the V9's (the SAM-M10Q carrier's UART on
# GPIO3/GPIO4, the mosaic's 1PPS on pin 5 = GPIO2), so one V8 image flies
# either receiver. The V8's default build stays u-blox-only. Neither auto
# image takes sdkconfig.defaults.rawnav: they fly GpsInsEKF, PSRAM off.
if(NOT DEFINED TR_GNSS_AUTO AND TR_BOARD_V9 AND NOT TR_GNSS_MOSAIC)
    set(TR_GNSS_AUTO ON)
endif()
if(TR_GNSS_AUTO)
    if(NOT TR_BOARD_V9 AND NOT TR_BOARD_V8)
        message(FATAL_ERROR "TR_GNSS_AUTO is for the V9 (-DTR_BOARD_V9=1) or the V8 (-DTR_BOARD_V8=1): their J1 takes either receiver")
    endif()
    add_compile_definitions(TR_GNSS_DRIVER_AUTO=1)
    # The V8 (legacy) sets no defaults list of its own: start it from the base
    # file, or the overlay alone would replace it and the build lose its P4
    # target.
    if(NOT SDKCONFIG_DEFAULTS)
        set(SDKCONFIG_DEFAULTS "sdkconfig.defaults")
    endif()
    list(APPEND SDKCONFIG_DEFAULTS "sdkconfig.defaults.mosaic")
    message(STATUS "flight_computer: GNSS driver auto (u-blox or mosaic-G5, found at boot: TR_GNSS_DRIVER_AUTO=1), overlay sdkconfig.defaults.mosaic")
elseif(TR_BOARD_V9 AND NOT TR_GNSS_MOSAIC)
    message(STATUS "flight_computer: GNSS driver u-blox only (-DTR_GNSS_AUTO=0)")
endif()
set(SDKCONFIG "${CMAKE_BINARY_DIR}/sdkconfig")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(flight_computer)
