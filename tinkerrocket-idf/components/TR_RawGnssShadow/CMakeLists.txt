# TR_RawGnssShadow: the raw-GNSS shadow filter (#1600 phase 4) -- TR_RawGnssEKF
# run beside the flight filter on its IMU input and the LC86G's raw epochs,
# logged as RAW_GNSS_EKF_MSG and never acted on; with its pad survey (PadSurvey,
# phase 5), the backup ionosphere; RawGnssMeasStage builds its epochs, in a
# GNSS task of their own on the flight computer (#1626). RawGnssShadow is pure C++;
# the flight computer's core-0 task (main.cpp) feeds it through ShadowLink.
# tests_cpp/test_raw_gnss_shadow.cpp compiles it directly.
idf_component_register(SRCS "RawGnssShadow.cpp" "RawGnssMeasStage.cpp" "PadSurvey.cpp" "SatJumpCheck.cpp"
                            "FixCheck.cpp" "EphemerisFrames.cpp" "HeadingFit.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES TR_RawGnssEKF TR_GNSSRaw TR_RocketComputerTypes)
