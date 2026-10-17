cmake_minimum_required(VERSION 3.16)
project(raw_replay C CXX)

# The Beetle's raw-GNSS navigation filter, offline: the firmware's own sources
# (RawGnssShadow, TR_RawGnssEKF, RawMeasBuilder, the LC86G's parser), compiled
# unmodified for the host, driven by raw_replay.cpp from one OC flight log's
# event stream (extract.py). See README.md.

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# SRC_ROOT: the tree whose firmware sources are compiled (default: this one). The options say what that tree
# has; the defaults are this tree's (the design flown since #1637). For an older tree: RAW_REPLAY_OLD=ON before the
# receiver profiles and the carrier (the rig's r20-r26 code, aef947c5), RAW_REPLAY_VCHECK=OFF before the vertical
# check (fc/gnss-heading-fuse 7966f2cb), RAW_REPLAY_LATCH=OFF before its latch (4755f128). DYNMEM and FITGATE are
# experiments the firmware does not carry: build them against a copy with experiments/*.patch applied (README.md).
set(SRC_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." CACHE PATH "the tree whose firmware sources are compiled")
option(RAW_REPLAY_OLD "the firmware tree has no receiver profile or carrier (aef947c5)" OFF)
option(RAW_REPLAY_VCHECK "the firmware tree has the vertical check and the boost window (fc/gnss-heading-fuse 7966f2cb)" ON)
option(RAW_REPLAY_DYNMEM "the firmware tree has the Doppler's settling (rr_dyn_tau_s: experiments/doppler_settling_fade.patch)" OFF)
option(RAW_REPLAY_LATCH "the firmware tree has the vertical check's latch (vert_release_mps)" ON)
option(RAW_REPLAY_FITGATE "the firmware tree has the Dopplers' fit gate (rr_fit_gate: experiments/doppler_fit_gate.patch)" OFF)
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
add_executable(raw_replay raw_replay.cpp ${CARRIER_SRC} ${RAW_REPLAY_EXTRA_SRC}
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
target_include_directories(raw_replay PRIVATE
    ${SHIM_DIR}
    ${LIB_DIR}/TR_RawGnssShadow
    ${LIB_DIR}/TR_RawGnssEKF
    ${LIB_DIR}/TR_GNSSRaw
    ${LIB_DIR}/TR_GNSSRaw/include
    ${LIB_DIR}/TR_GNSSReceiverLC86_Serial
    ${LIB_DIR}/TR_RocketComputerTypes
    ${FC_MAIN}
    ${RAW_REPLAY_EXTRA_INC}
)
target_compile_definitions(raw_replay PRIVATE RAW_REPLAY_OLD=$<BOOL:${RAW_REPLAY_OLD}>
    RAW_REPLAY_VCHECK=$<BOOL:${RAW_REPLAY_VCHECK}>
    RAW_REPLAY_DYNMEM=$<BOOL:${RAW_REPLAY_DYNMEM}>
    RAW_REPLAY_LATCH=$<BOOL:${RAW_REPLAY_LATCH}>
    RAW_REPLAY_FITGATE=$<BOOL:${RAW_REPLAY_FITGATE}>)
