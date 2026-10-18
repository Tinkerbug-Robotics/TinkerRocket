# The raw-GNSS filter's core switches for the mosaic-G5 (fc/v9raw-core): each
# at its default the Beetle's filter exactly, and at the mosaic's values what
# the build plan says. The mosaic-G5's profile and the Beetle's tripwires are
# test_receiver_profile's (tests_cpp/CMakeLists.txt).

# ---------- test_raw_gnss_ekf_v9 ----------
# TR_RawGnssEKF: the unaided twin (att_gnss_off: GNSS kept off the attitude
# and the gyro bias only), the Doppler's own refusal count at 10 Hz
# (reject_persist_rr, the boost screen's let-in with it), the pseudorange off
# (setFusePr), a restored flight's sigmas and accelerometer bias.
add_executable(test_raw_gnss_ekf_v9 test_raw_gnss_ekf_v9.cpp
    ${LIB_DIR}/TR_GNSSRaw/atmo.c
    ${LIB_DIR}/TR_RawGnssEKF/TR_RawGnssEKF.cpp
    ${LIB_DIR}/TR_RawGnssEKF/TR_RawGnssCarrier.cpp
)
target_include_directories(test_raw_gnss_ekf_v9 PRIVATE
    ${LIB_DIR}/TR_GNSSRaw
    ${LIB_DIR}/TR_GNSSRaw/include
    ${LIB_DIR}/TR_RawGnssEKF
)
target_link_libraries(test_raw_gnss_ekf_v9 GTest::gtest_main)
gtest_discover_tests(test_raw_gnss_ekf_v9)

# ---------- test_raw_gnss_shadow_v9 ----------
# RawGnssShadow and LevelGate at 10 Hz, through the mosaic-G5's profile on the
# synthetic sky (fixtures/raw_meas_golden.h): the pseudorange once a second
# with gnss_aided held (on the first epoch of each second fused, a whole
# second's epoch lost or dropped or not), the Doppler's stress the window's
# peak a second after burnout (a falling thrust too), the heading fit an epoch
# a second, a restored flight's sigmas and accelerometer bias, levelling on
# fused epochs only (empty epochs under a chute), the steady test on epochs a
# second apart, and the profile's satellite cap reaching the GNSS task's stage
# (skipped until the stage has one).
add_executable(test_raw_gnss_shadow_v9 test_raw_gnss_shadow_v9.cpp
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
    ${LIB_DIR}/TR_GNSSRaw/RawMeasBuilderLite.cpp
)
target_include_directories(test_raw_gnss_shadow_v9 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${SHIM_DIR}
    ${LIB_DIR}/TR_RawGnssShadow
    ${LIB_DIR}/TR_RawGnssEKF
    ${LIB_DIR}/TR_GNSSRaw
    ${LIB_DIR}/TR_GNSSRaw/include
    ${LIB_DIR}/TR_RocketComputerTypes
)
target_link_libraries(test_raw_gnss_shadow_v9 GTest::gtest_main)
gtest_discover_tests(test_raw_gnss_shadow_v9)
