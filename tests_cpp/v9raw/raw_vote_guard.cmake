# ---------- test_raw_vote_guard (mosaic-only build) ----------
# With the raw-GNSS filter as the only navigation filter there is no second
# filter to cross-check it, so after burnout and before apogee its vertical
# velocity is checked against the barometer's climb rate: reading more than
# 10 m/s below it for a continuous second withholds its velocity and pitch
# apogee votes for the rest of the flight.
# Pure header in the flight_computer project beside baro_gate_policy.h.
add_executable(test_raw_vote_guard test_raw_vote_guard.cpp)
target_include_directories(test_raw_vote_guard PRIVATE
    ${SHIM_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../tinkerrocket-idf/projects/flight_computer/main
)
target_link_libraries(test_raw_vote_guard GTest::gtest_main)
gtest_discover_tests(test_raw_vote_guard)
