# ---------- test_raw_gnss_antenna ----------
# The antenna's offset where the rocket's owner measured it (TR_RawGnssEKF.h
# lever_on, RawGnssShadow.h "The antenna's offset"): the antenna's offset and
# velocity kept beside the history and taken at each measurement's instant,
# the Dopplers, pseudoranges and the change check's changes predicted at the
# antenna, the offset's roll terms observed or fused, and the owner's setting
# (BLE cmd 81, the board's frame) in the filter's. Off, the Beetle's filter.
# The shadow's side is in test_raw_gnss_shadow_v9.
add_executable(test_raw_gnss_antenna test_raw_gnss_antenna.cpp
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
target_include_directories(test_raw_gnss_antenna PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${SHIM_DIR}
    ${LIB_DIR}/TR_RawGnssShadow
    ${LIB_DIR}/TR_RawGnssEKF
    ${LIB_DIR}/TR_GNSSRaw
    ${LIB_DIR}/TR_GNSSRaw/include
    ${LIB_DIR}/TR_RocketComputerTypes
)
target_link_libraries(test_raw_gnss_antenna GTest::gtest_main)
gtest_discover_tests(test_raw_gnss_antenna)
