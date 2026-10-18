cmake_minimum_required(VERSION 3.16)
project(raw_replay C CXX)

# The Beetle's raw-GNSS navigation filter, offline: the firmware's own sources
# (RawGnssShadow, TR_RawGnssEKF, RawMeasBuilder, the LC86G's parser), compiled
# unmodified for the host, driven by raw_replay.cpp from one OC flight log's
# event stream (extract.py). See README.md.
#
# Two binaries from the same sources: raw_replay, with the Beetle's 64-entry
# ephemeris store, and raw_replay_mosaic, with the mosaic-only build's 128
# (TR_EPH_CAPACITY, set build-wide as the flight computer's CMake sets it).
# Each replays either receiver, and says so when it is not the one its store
# was built for.

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# SRC_ROOT: the tree whose firmware sources are compiled (default: this one). The options say what that tree
# has; the defaults are this tree's (the design flown since #1637). For an older tree: RAW_REPLAY_OLD=ON before the
# receiver profiles and the carrier (the rig's r20-r26 code, aef947c5), RAW_REPLAY_VCHECK=OFF before the vertical
# check (fc/gnss-heading-fuse 7966f2cb), RAW_REPLAY_LATCH=OFF before its latch (4755f128), RAW_REPLAY_MOSAIC=OFF
# before the mosaic-G5's decoders, the stage's addEphemeris and the unaided switch (fc/v9-raw-primary), RAW_REPLAY_ROT=OFF
# before the turning body's allowances (exp/v9-robust), RAW_REPLAY_DVHDG=OFF before the change check's heading
# allowance for a launch with no magnetometer heading (fc/v9raw-dopp after 357e7021), RAW_REPLAY_ANTENNA=OFF before
# the antenna's offset where it is known (fc/v9-antenna-offset). DYNMEM and
# FITGATE are experiments the firmware does not carry: build them against a copy with experiments/*.patch applied
# (README.md).
set(SRC_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." CACHE PATH "the tree whose firmware sources are compiled")
option(RAW_REPLAY_OLD "the firmware tree has no receiver profile or carrier (aef947c5)" OFF)
option(RAW_REPLAY_VCHECK "the firmware tree has the vertical check and the boost window (fc/gnss-heading-fuse 7966f2cb)" ON)
option(RAW_REPLAY_DYNMEM "the firmware tree has the Doppler's settling (rr_dyn_tau_s: experiments/doppler_settling_fade.patch)" OFF)
option(RAW_REPLAY_LATCH "the firmware tree has the vertical check's latch (vert_release_mps)" ON)
option(RAW_REPLAY_FITGATE "the firmware tree has the Dopplers' fit gate (rr_fit_gate: experiments/doppler_fit_gate.patch)" OFF)
option(RAW_REPLAY_MOSAIC "the firmware tree has the mosaic-G5 path (SBF decoders, addEphemeris, att_gnss_off, the raw vote guard)" ON)
option(RAW_REPLAY_DVCHECK "the firmware tree has the Doppler change check (RawGnssShadow::Config::dv_check, exp/v9-dvcheck)" ON)
option(RAW_REPLAY_HDGOBS "the firmware tree has the heading gain by its observability (hdg_obs_mode, exp/v9-observ)" ON)
option(RAW_REPLAY_ROT "the firmware tree has the turning body's allowances (rot_lever_*, att_hold_*: exp/v9-robust)" ON)
option(RAW_REPLAY_DVHDG "the firmware tree has the change check's heading allowance for a launch with no magnetometer heading (dv_hdg_unset_deg)" ON)
option(RAW_REPLAY_ANTENNA "the firmware tree has the antenna's offset where it is known (lever_on, applyAntennaOffset: fc/v9-antenna-offset)" ON)
if(RAW_REPLAY_OLD AND RAW_REPLAY_MOSAIC)
    message(STATUS "raw_replay: RAW_REPLAY_OLD turns RAW_REPLAY_MOSAIC off")
    set(RAW_REPLAY_MOSAIC OFF)
endif()
if(RAW_REPLAY_OLD AND RAW_REPLAY_DVCHECK)
    set(RAW_REPLAY_DVCHECK OFF)
endif()
if(NOT RAW_REPLAY_DVCHECK AND RAW_REPLAY_DVHDG)
    set(RAW_REPLAY_DVHDG OFF)
endif()
if(NOT RAW_REPLAY_MOSAIC AND RAW_REPLAY_HDGOBS)
    set(RAW_REPLAY_HDGOBS OFF)
endif()
if(RAW_REPLAY_OLD AND RAW_REPLAY_ROT)
    message(STATUS "raw_replay: RAW_REPLAY_OLD turns RAW_REPLAY_ROT off")
    set(RAW_REPLAY_ROT OFF)
endif()
if(NOT RAW_REPLAY_ROT AND RAW_REPLAY_ANTENNA)
    set(RAW_REPLAY_ANTENNA OFF)
endif()
# the mosaic-only image's flight safeguards in one call (gnss_raw::applyMosaicFlightSafeguards, ReceiverProfileApply.h):
# a tree with all three of the features it sets
option(RAW_REPLAY_MOSAIC_SAFEGUARDS "the firmware tree sets the mosaic's flight safeguards in one call (applyMosaicFlightSafeguards)" ON)
if(RAW_REPLAY_MOSAIC_SAFEGUARDS AND NOT (RAW_REPLAY_MOSAIC AND RAW_REPLAY_DVCHECK AND RAW_REPLAY_HDGOBS AND RAW_REPLAY_ROT))
    message(STATUS "raw_replay: RAW_REPLAY_MOSAIC_SAFEGUARDS off (one of RAW_REPLAY_MOSAIC, DVCHECK, HDGOBS and ROT is "
                   "off): --receiver mosaic will not apply the image's flight safeguards")
    set(RAW_REPLAY_MOSAIC_SAFEGUARDS OFF)
endif()
set(LIB_DIR "${SRC_ROOT}/tinkerrocket-idf/components")
set(FC_MAIN "${SRC_ROOT}/tinkerrocket-idf/projects/flight_computer/main")
set(SHIM_DIR "${SRC_ROOT}/tests_cpp/host_shim")
# A later tree whose filter needs a source file or include directory this list does not name: its paths, relative
# to SRC_ROOT (';'-separated). bitident.sh builds this file as it was frozen, so a tree that grows is linked here,
# not by editing it.
set(RAW_REPLAY_EXTRA_SRC "" CACHE STRING "more firmware sources, relative to SRC_ROOT")
set(RAW_REPLAY_EXTRA_INC "" CACHE STRING "more include directories, relative to SRC_ROOT")
list(TRANSFORM RAW_REPLAY_EXTRA_SRC PREPEND "${SRC_ROOT}/")
list(TRANSFORM RAW_REPLAY_EXTRA_INC PREPEND "${SRC_ROOT}/")

set(CARRIER_SRC ${LIB_DIR}/TR_RawGnssEKF/TR_RawGnssCarrier.cpp ${LIB_DIR}/TR_GNSSRaw/RawMeasCarrier.cpp)
if(RAW_REPLAY_OLD)
    set(CARRIER_SRC "")
endif()
# The mosaic-G5's path: its SBF decoders (the driver's and the GNSS task's), the RINEX reader, and what the flight
# loop's apogee quorum runs on (TR_KinematicChecks, the start's attitude and the magnetic model)
set(MOSAIC_SRC "")
set(MOSAIC_INC "")
if(RAW_REPLAY_MOSAIC)
    set(MOSAIC_SRC
        rinex_nav.cpp
        ${LIB_DIR}/TR_GNSSReceiverMosaic_Serial/SbfMeas.cpp
        ${LIB_DIR}/TR_GNSSReceiverMosaic_Serial/SbfNav.cpp
        ${LIB_DIR}/TR_GNSSReceiverMosaic_Serial/SbfParser.cpp
        ${LIB_DIR}/TR_KinematicChecks/TR_KinematicChecks.cpp
        ${LIB_DIR}/TR_GeoMag/TR_GeoMag.cpp
        ${LIB_DIR}/TR_Orientation/TR_Orientation.cpp)
    set(MOSAIC_INC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${LIB_DIR}/TR_GNSSReceiverMosaic_Serial
        ${LIB_DIR}/TR_KinematicChecks
        ${LIB_DIR}/TR_GpsInsEKF
        ${LIB_DIR}/TR_GeoMag
        ${LIB_DIR}/TR_Orientation)
endif()

function(raw_replay_target name)
    add_executable(${name} raw_replay.cpp ${CARRIER_SRC} ${MOSAIC_SRC} ${RAW_REPLAY_EXTRA_SRC}
        ${LIB_DIR}/TR_RawGnssShadow/RawGnssShadow.cpp
        ${LIB_DIR}/TR_RawGnssShadow/RawGnssMeasStage.cpp
        ${LIB_DIR}/TR_RawGnssShadow/PadSurvey.cpp
        ${LIB_DIR}/TR_RawGnssShadow/SatJumpCheck.cpp
        ${LIB_DIR}/TR_RawGnssShadow/FixCheck.cpp
        ${LIB_DIR}/TR_RawGnssShadow/EphemerisFrames.cpp
        ${LIB_DIR}/TR_RawGnssShadow/HeadingFit.cpp
        ${LIB_DIR}/TR_RawGnssEKF/TR_RawGnssEKF.cpp
        ${LIB_DIR}/TR_GNSSRaw/eph.c
        ${LIB_DIR}/TR_GNSSRaw/atmo.c
        ${LIB_DIR}/TR_GNSSRaw/RtcmEphemeris.cpp
        ${LIB_DIR}/TR_GNSSRaw/EphemerisStore.cpp
        ${LIB_DIR}/TR_GNSSRaw/RawMeasBuilder.cpp
        ${LIB_DIR}/TR_GNSSReceiverLC86_Serial/Lc86Parser.cpp
        ${LIB_DIR}/TR_GNSSReceiverLC86_Serial/Rtcm3Msm.cpp
    )
    target_include_directories(${name} PRIVATE
        ${SHIM_DIR}
        ${LIB_DIR}/TR_RawGnssShadow
        ${LIB_DIR}/TR_RawGnssEKF
        ${LIB_DIR}/TR_GNSSRaw
        ${LIB_DIR}/TR_GNSSRaw/include
        ${LIB_DIR}/TR_GNSSReceiverLC86_Serial
        ${LIB_DIR}/TR_RocketComputerTypes
        ${FC_MAIN}
        ${MOSAIC_INC}
        ${RAW_REPLAY_EXTRA_INC}
    )
    target_compile_definitions(${name} PRIVATE RAW_REPLAY_OLD=$<BOOL:${RAW_REPLAY_OLD}>
        RAW_REPLAY_VCHECK=$<BOOL:${RAW_REPLAY_VCHECK}>
        RAW_REPLAY_DYNMEM=$<BOOL:${RAW_REPLAY_DYNMEM}>
        RAW_REPLAY_LATCH=$<BOOL:${RAW_REPLAY_LATCH}>
        RAW_REPLAY_FITGATE=$<BOOL:${RAW_REPLAY_FITGATE}>
        RAW_REPLAY_MOSAIC=$<BOOL:${RAW_REPLAY_MOSAIC}>
        RAW_REPLAY_DVCHECK=$<BOOL:${RAW_REPLAY_DVCHECK}>
        RAW_REPLAY_HDGOBS=$<BOOL:${RAW_REPLAY_HDGOBS}>
        RAW_REPLAY_ROT=$<BOOL:${RAW_REPLAY_ROT}>
        RAW_REPLAY_DVHDG=$<BOOL:${RAW_REPLAY_DVHDG}>
        RAW_REPLAY_ANTENNA=$<BOOL:${RAW_REPLAY_ANTENNA}>
        RAW_REPLAY_MOSAIC_SAFEGUARDS=$<BOOL:${RAW_REPLAY_MOSAIC_SAFEGUARDS}>)
endfunction()

raw_replay_target(raw_replay)
if(RAW_REPLAY_MOSAIC)
    raw_replay_target(raw_replay_mosaic)
    target_compile_definitions(raw_replay_mosaic PRIVATE TR_EPH_CAPACITY=128)
endif()
