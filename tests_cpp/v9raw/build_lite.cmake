# ---------- test_raw_meas_lite ----------
# The builder's cheaper build (RawMeasBuilder.h, Config::lite; the receiver
# profile's build_lite, off everywhere but a bench image): off, it is the
# full build; on, each second's first epoch gets the full build's corrections
# and the others keep them, the orbit solves spread over their 5 s, the noise
# is the full model's, and the GNSS task's stage builds the same picks. The
# Beetle's two MSM7 epochs (fixtures/raw_meas_golden.h) stepped at 10 Hz.
add_executable(test_raw_meas_lite test_raw_meas_lite.cpp
    ${LIB_DIR}/TR_RawGnssShadow/RawGnssMeasStage.cpp
    ${LIB_DIR}/TR_GNSSRaw/eph.c
    ${LIB_DIR}/TR_GNSSRaw/atmo.c
    ${LIB_DIR}/TR_GNSSRaw/RtcmEphemeris.cpp
    ${LIB_DIR}/TR_GNSSRaw/EphemerisStore.cpp
    ${LIB_DIR}/TR_GNSSRaw/RawMeasBuilder.cpp
    ${LIB_DIR}/TR_GNSSRaw/RawMeasCarrier.cpp
    ${LIB_DIR}/TR_GNSSRaw/RawMeasBuilderLite.cpp
    ${LIB_DIR}/TR_GNSSReceiverLC86_Serial/Lc86Parser.cpp
    ${LIB_DIR}/TR_GNSSReceiverLC86_Serial/Rtcm3Msm.cpp
)
target_include_directories(test_raw_meas_lite PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${SHIM_DIR}
    ${LIB_DIR}/TR_RawGnssShadow
    ${LIB_DIR}/TR_RawGnssEKF
    ${LIB_DIR}/TR_GNSSRaw
    ${LIB_DIR}/TR_GNSSRaw/include
    ${LIB_DIR}/TR_GNSSReceiverLC86_Serial
    ${LIB_DIR}/TR_RocketComputerTypes
)
target_link_libraries(test_raw_meas_lite GTest::gtest_main)
gtest_discover_tests(test_raw_meas_lite)
