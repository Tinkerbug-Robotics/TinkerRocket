# ---------- test_sbf_nav ----------
# The mosaic-G5's navigation decoders (TR_GNSSReceiverMosaic_Serial/
# SbfNav.cpp): GPSNav, GALNav and BDSNav to gps_eph_t, each checked against
# the LC86G's RTCM decode of the same ephemeris (fixtures/raw_meas_golden.h),
# what is refused (Galileo F/NAV, BeiDou GEO, a GPS cutover, do-not-use
# fields), and GPSIon to the 8 Klobuchar bytes. With TR_SBF_PRIVATE_CAPTURE
# set, every nav block of the 10/16 V9 captures too (statistics only).
add_executable(test_sbf_nav test_sbf_nav.cpp
    ${LIB_DIR}/TR_GNSSReceiverMosaic_Serial/SbfNav.cpp
    ${LIB_DIR}/TR_GNSSReceiverMosaic_Serial/SbfMeas.cpp
    ${LIB_DIR}/TR_GNSSReceiverMosaic_Serial/SbfParser.cpp
    ${LIB_DIR}/TR_GNSSRaw/eph.c
    ${LIB_DIR}/TR_GNSSRaw/atmo.c
    ${LIB_DIR}/TR_GNSSRaw/RtcmEphemeris.cpp
    ${LIB_DIR}/TR_GNSSRaw/EphemerisStore.cpp
)
target_include_directories(test_sbf_nav PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${SHIM_DIR}
    ${LIB_DIR}/TR_GNSSReceiverMosaic_Serial
    ${LIB_DIR}/TR_GNSSRaw
    ${LIB_DIR}/TR_GNSSRaw/include
    ${LIB_DIR}/TR_RocketComputerTypes
)
target_link_libraries(test_sbf_nav GTest::gtest_main)
gtest_discover_tests(test_sbf_nav)
