# ---------- the replay harness's mosaic-G5 pieces (tools/raw_replay) ----------
# The parts of tools/raw_replay that stand between a V8/V9 log and the
# firmware's filter, tested on their own; the harness binary itself is built
# from tools/raw_replay/CMakeLists.txt.
set(RAW_REPLAY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../tools/raw_replay)

# ---------- test_rinex_nav ----------
# The day's broadcast RINEX 3 file as gps_eph_t, by SbfNav's conventions:
# every golden RTCM ephemeris written as its RINEX record comes back to 1 cm
# and 1 ns, F/NAV, GEO and other systems are left out, the Galileo health
# marks, dates and availability (the rule preloaded nav blocks are filed by). With TR_SBF_PRIVATE_CAPTURE and
# TR_RINEX_NAV set, the 10/16 captures' own nav blocks against the file.
add_executable(test_rinex_nav test_rinex_nav.cpp
    ${RAW_REPLAY_DIR}/rinex_nav.cpp
    ${LIB_DIR}/TR_GNSSReceiverMosaic_Serial/SbfNav.cpp
    ${LIB_DIR}/TR_GNSSReceiverMosaic_Serial/SbfMeas.cpp
    ${LIB_DIR}/TR_GNSSReceiverMosaic_Serial/SbfParser.cpp
    ${LIB_DIR}/TR_GNSSRaw/eph.c
    ${LIB_DIR}/TR_GNSSRaw/atmo.c
    ${LIB_DIR}/TR_GNSSRaw/RtcmEphemeris.cpp
)
target_include_directories(test_rinex_nav PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${RAW_REPLAY_DIR}
    ${SHIM_DIR}
    ${LIB_DIR}/TR_GNSSReceiverMosaic_Serial
    ${LIB_DIR}/TR_GNSSRaw
    ${LIB_DIR}/TR_GNSSRaw/include
    ${LIB_DIR}/TR_RocketComputerTypes
)
target_link_libraries(test_rinex_nav GTest::gtest_main)
gtest_discover_tests(test_rinex_nav)

# ---------- test_raw_replay_mosaic ----------
# What the harness does with each SBF block (mosaic_feed.h): epochs on the
# fused grid, their GPS time from the anchoring PVT's clock bias through the
# clock jumps, a lost MeasEpoch's PVT anchoring nothing, nav blocks kept as
# the driver keeps them, the one ionosphere rule, PVTGeodetic as the fix, and
# the driver's time tag (FINETIME, a 1PPS pairing within 120 s, the line).
add_executable(test_raw_replay_mosaic test_raw_replay_mosaic.cpp
    ${LIB_DIR}/TR_GNSSReceiverMosaic_Serial/SbfNav.cpp
    ${LIB_DIR}/TR_GNSSReceiverMosaic_Serial/SbfMeas.cpp
    ${LIB_DIR}/TR_GNSSReceiverMosaic_Serial/SbfParser.cpp
    ${LIB_DIR}/TR_GNSSRaw/eph.c
    ${LIB_DIR}/TR_GNSSRaw/RtcmEphemeris.cpp
)
target_include_directories(test_raw_replay_mosaic PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${RAW_REPLAY_DIR}
    ${SHIM_DIR}
    ${LIB_DIR}/TR_GNSSReceiverMosaic_Serial
    ${LIB_DIR}/TR_GNSSRaw
    ${LIB_DIR}/TR_GNSSRaw/include
    ${LIB_DIR}/TR_RawGnssShadow
    ${LIB_DIR}/TR_RocketComputerTypes
    ${CMAKE_CURRENT_SOURCE_DIR}/../tinkerrocket-idf/projects/flight_computer/main   # raw_iono_rule.h, where it is
)
target_link_libraries(test_raw_replay_mosaic GTest::gtest_main)
gtest_discover_tests(test_raw_replay_mosaic)

# ---------- test_raw_replay_votes ----------
# The flight computer's apogee quorum as the harness runs it (flight_votes.h):
# TR_KinematicChecks, GnssAscentGate, the burnout detector, the raw vote guard
# and RawNavAdapter on a synthetic flight -- the call just after the true
# apogee on the truth, the guard latching on a filter reading low, a
# coherent receiver fault calling early without the guard and not with it,
# the plan's coherent coast ramp as the guard's gap (a raw vote above
# +2 m/s, the guard silent), the filter's tick flags from the quorum (its
# levelling in flight from the quorum's call), and with state_from_quorum
# the launch and landing from its own kinematics. The truth read from a file
# (truth_nav.h, raw_replay --truth-nav) votes exactly as the truth handed
# over, and a bad file is refused.
add_executable(test_raw_replay_votes test_raw_replay_votes.cpp
    ${LIB_DIR}/TR_KinematicChecks/TR_KinematicChecks.cpp
)
target_include_directories(test_raw_replay_votes PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${RAW_REPLAY_DIR}
    ${SHIM_DIR}
    ${LIB_DIR}/TR_KinematicChecks
    ${LIB_DIR}/TR_GpsInsEKF
    ${LIB_DIR}/TR_RawGnssShadow
    ${LIB_DIR}/TR_RawGnssEKF
    ${LIB_DIR}/TR_GNSSRaw
    ${LIB_DIR}/TR_GNSSRaw/include
    ${LIB_DIR}/TR_RocketComputerTypes
    ${CMAKE_CURRENT_SOURCE_DIR}/../tinkerrocket-idf/projects/flight_computer/main
)
target_link_libraries(test_raw_replay_votes GTest::gtest_main)
gtest_discover_tests(test_raw_replay_votes)
