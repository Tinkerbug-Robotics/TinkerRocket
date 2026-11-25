# ---------- test_mosaic_multiband ----------
# The multi-band combiner (TR_GNSSReceiverMosaic_Serial/MosaicMultiband.h: the
# mosaic image's driver, config.h TR_RAW_MULTIBAND, and the harness's
# --rr-multiband): off and with nothing to
# join it is the image's adapter bit for bit, on every anchor field but the
# range rate is; the scaling, the inverse-variance weights (each band's own
# C/N0 offset), the wind-up referred to the anchor's wavelength (or told as
# the weighted one) with the weighted, smoothed estimate or the gyro's, the
# screen and the anchor's vote, the join rule (lock, hysteresis, channel, the
# half-cycle flag gating nothing), composition changes, faults on the anchor
# alone and the drift guard, the review's three defects (the estimate's capture
# by one satellite, the guard's hold, a fault common to the anchors or to one
# system's), the pairs on combined ends, the effective C/N0, and what the
# image flies (mosaicMbFlight, the gyro's wind-up mbWindupFromGyro). With
# TR_SBF_PRIVATE_CAPTURE set, real captures for the identities (statistics
# only).
add_executable(test_mosaic_multiband test_mosaic_multiband.cpp
    ${LIB_DIR}/TR_GNSSReceiverMosaic_Serial/SbfNav.cpp
    ${LIB_DIR}/TR_GNSSReceiverMosaic_Serial/SbfMeas.cpp
    ${LIB_DIR}/TR_GNSSReceiverMosaic_Serial/SbfParser.cpp
    ${LIB_DIR}/TR_GNSSRaw/eph.c
    ${LIB_DIR}/TR_GNSSRaw/RtcmEphemeris.cpp
)
target_include_directories(test_mosaic_multiband PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../tools/raw_replay
    ${SHIM_DIR}
    ${LIB_DIR}/TR_GNSSReceiverMosaic_Serial
    ${LIB_DIR}/TR_GNSSRaw
    ${LIB_DIR}/TR_GNSSRaw/include
    ${LIB_DIR}/TR_RawGnssShadow
    ${LIB_DIR}/TR_RocketComputerTypes
    ${CMAKE_CURRENT_SOURCE_DIR}/../tinkerrocket-idf/projects/flight_computer/main   # raw_iono_rule.h, where it is
)
target_link_libraries(test_mosaic_multiband GTest::gtest_main)
gtest_discover_tests(test_mosaic_multiband)
