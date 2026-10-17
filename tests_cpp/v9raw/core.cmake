# The raw-GNSS filter's core switches for the mosaic-G5 (fc/v9raw-core): each
# at its default the Beetle's filter exactly, and at the mosaic's values what
# the build plan says.

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

