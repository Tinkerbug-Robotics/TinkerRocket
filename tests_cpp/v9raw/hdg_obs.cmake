# ---------- test_raw_gnss_hdg_obs (exp/v9-observ) ----------
# TR_RawGnssEKF's hdg_obs_mode: the GNSS rows' gain on the heading scaled by
# how well the moment observes it -- by the horizontal specific force, by the
# heading's information from it, or closed in the coast's first seconds --
# with the boost left open, today's filter with the mode off or a share of
# 1, the unaided twin unchanged, and RawGnssShadow's heading_obs_scope.
add_executable(test_raw_gnss_hdg_obs test_raw_gnss_hdg_obs.cpp
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
    ${LIB_DIR}/TR_GNSSReceiverLC86_Serial/Lc86Parser.cpp
    ${LIB_DIR}/TR_GNSSReceiverLC86_Serial/Rtcm3Msm.cpp
)
target_include_directories(test_raw_gnss_hdg_obs PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${SHIM_DIR}
    ${LIB_DIR}/TR_RawGnssShadow
    ${LIB_DIR}/TR_RawGnssEKF
    ${LIB_DIR}/TR_GNSSRaw
    ${LIB_DIR}/TR_GNSSRaw/include
    ${LIB_DIR}/TR_GNSSReceiverLC86_Serial
    ${LIB_DIR}/TR_RocketComputerTypes
)
target_link_libraries(test_raw_gnss_hdg_obs GTest::gtest_main)
gtest_discover_tests(test_raw_gnss_hdg_obs)
