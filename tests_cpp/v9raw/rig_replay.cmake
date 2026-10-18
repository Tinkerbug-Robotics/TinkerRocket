# ---------- test_sim_replay_epochs (the V9 mosaic image's rig replay) ----------
# A TR_SIM_REPLAY_BLOB build of the mosaic-only image sets the replay's clock
# from the mosaic-G5's timed raw epochs (ReplayClock::onEpoch: its driver hands
# no timed fix), and reads the carried 500 Hz flight at each of the V9 IMU's
# own instants (3780-7680 Hz). Pure SimReplay, as test_sim_replay.
add_executable(test_sim_replay_epochs test_sim_replay_epochs.cpp
    ${LIB_DIR}/TR_Sensor_Collector_Sim/SimReplay.cpp
)
target_include_directories(test_sim_replay_epochs PRIVATE
    ${LIB_DIR}/TR_Sensor_Collector_Sim
)
target_link_libraries(test_sim_replay_epochs GTest::gtest_main)
gtest_discover_tests(test_sim_replay_epochs)

# ---------- test_sim_high_g_bias (the rig replay's high-g counts) ----------
# The replayed high-g counts carry the converter's high-g bias, so where the
# low-g rail hands over the flight's own force comes out (SimHighGBias.h),
# checked through SensorConverter itself.
add_executable(test_sim_high_g_bias test_sim_high_g_bias.cpp
    ${LIB_DIR}/TR_Sensor_Data_Converter/TR_Sensor_Data_Converter.cpp
    ${LIB_DIR}/TR_Orientation/TR_Orientation.cpp
)
target_include_directories(test_sim_high_g_bias PRIVATE
    ${SHIM_DIR}
    ${LIB_DIR}/TR_Sensor_Data_Converter
    ${LIB_DIR}/TR_RocketComputerTypes
    ${LIB_DIR}/TR_Orientation
    ${LIB_DIR}/TR_Sensor_Collector_Sim
)
target_link_libraries(test_sim_high_g_bias GTest::gtest_main)
gtest_discover_tests(test_sim_high_g_bias)
