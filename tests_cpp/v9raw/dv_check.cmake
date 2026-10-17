# ---------- test_doppler_change (exp/v9-dvcheck) ----------
# The Doppler change check (RawGnssShadow's dv_check, off by default): its
# arithmetic (TR_RawGnssEKF/DopplerChange.h: the receiver's change and the
# clock's fitted to each satellite's, the satellites left out on their own,
# the epoch's change against the IMU's with the attitude, scale, bias and
# lever-arm allowances), the EKF's rows (RawGnssEKF::dopplerChange: the
# filter's own velocity drops out), and the shadow acting on them (observe is
# the filter bit for bit; a stale satellite and a stale receiver's tail held
# out; the window's sum sees a drift no epoch does).
add_executable(test_doppler_change test_doppler_change.cpp
    ${LIB_DIR}/TR_RawGnssShadow/RawGnssShadow.cpp
    ${LIB_DIR}/TR_RawGnssShadow/RawGnssMeasStage.cpp
    ${LIB_DIR}/TR_RawGnssShadow/PadSurvey.cpp
    ${LIB_DIR}/TR_RawGnssShadow/SatJumpCheck.cpp
    ${LIB_DIR}/TR_RawGnssShadow/FixCheck.cpp
    ${LIB_DIR}/TR_RawGnssShadow/EphemerisFrames.cpp
    ${LIB_DIR}/TR_RawGnssShadow/HeadingFit.cpp
    ${LIB_DIR}/TR_RawGnssEKF/TR_RawGnssEKF.cpp
    ${LIB_DIR}/TR_RawGnssEKF/TR_RawGnssCarrier.cpp
    ${LIB_DIR}/TR_GNSSRaw/eph.c
    ${LIB_DIR}/TR_GNSSRaw/atmo.c
    ${LIB_DIR}/TR_GNSSRaw/RtcmEphemeris.cpp
    ${LIB_DIR}/TR_GNSSRaw/EphemerisStore.cpp
    ${LIB_DIR}/TR_GNSSRaw/RawMeasBuilder.cpp
    ${LIB_DIR}/TR_GNSSRaw/RawMeasCarrier.cpp
)
target_include_directories(test_doppler_change PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${SHIM_DIR}
    ${LIB_DIR}/TR_RawGnssShadow
    ${LIB_DIR}/TR_RawGnssEKF
    ${LIB_DIR}/TR_GNSSRaw
    ${LIB_DIR}/TR_GNSSRaw/include
    ${LIB_DIR}/TR_RocketComputerTypes
)
target_link_libraries(test_doppler_change GTest::gtest_main)
gtest_discover_tests(test_doppler_change)
