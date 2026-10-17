# ---------- test_raw_replay_sim ----------
# A sim's epochs through the GNSS task's stage, as raw_replay's 'X' events
# take them (tools/raw_replay/sim_epoch.h; Grid B): on the same sky, epoch
# after epoch, the sim path picks the satellites the stage's own offer()
# builds, with the same sigmas, lock ages and ionosphere choice and the same
# counts -- the cap's hold, a relock past the profile's gap, the C/N0 floor;
# the elevation mask and a stale satellite's extra Doppler sigma; untimed and
# idle epochs seen, not built.
add_executable(test_raw_replay_sim test_raw_replay_sim.cpp
    ${LIB_DIR}/TR_RawGnssShadow/RawGnssMeasStage.cpp
    ${LIB_DIR}/TR_GNSSRaw/eph.c
    ${LIB_DIR}/TR_GNSSRaw/atmo.c
    ${LIB_DIR}/TR_GNSSRaw/RtcmEphemeris.cpp
    ${LIB_DIR}/TR_GNSSRaw/EphemerisStore.cpp
    ${LIB_DIR}/TR_GNSSRaw/RawMeasBuilder.cpp
    ${LIB_DIR}/TR_GNSSRaw/RawMeasCarrier.cpp
)
target_include_directories(test_raw_replay_sim PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../tools/raw_replay
    ${SHIM_DIR}
    ${LIB_DIR}/TR_RawGnssShadow
    ${LIB_DIR}/TR_RawGnssEKF
    ${LIB_DIR}/TR_GNSSRaw
    ${LIB_DIR}/TR_GNSSRaw/include
    ${LIB_DIR}/TR_RocketComputerTypes
)
target_link_libraries(test_raw_replay_sim GTest::gtest_main)
gtest_discover_tests(test_raw_replay_sim)
