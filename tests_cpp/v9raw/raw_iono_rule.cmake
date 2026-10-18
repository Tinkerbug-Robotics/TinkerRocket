# ---------- test_raw_iono_rule (mosaic-only build) ----------
# The mosaic-only image's raw-GNSS filter has two broadcast ionospheres: the
# app's set from a pad calibration and the receiver's own GPSIon. One rule
# picks between them (no older day; the receiver's on a tie; the same bytes
# never applied twice). Pure header in the flight_computer project.
add_executable(test_raw_iono_rule test_raw_iono_rule.cpp)
target_include_directories(test_raw_iono_rule PRIVATE
    ${SHIM_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../tinkerrocket-idf/projects/flight_computer/main
)
target_link_libraries(test_raw_iono_rule GTest::gtest_main)
gtest_discover_tests(test_raw_iono_rule)
