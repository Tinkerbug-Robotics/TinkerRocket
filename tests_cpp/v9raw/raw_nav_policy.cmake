# ---------- test_raw_nav_policy (mosaic-only build) ----------
# The mosaic-only image's rules for its one navigation filter: no filter on
# the third boot after two crash resets, the transonic lockout begun only
# under thrust and never on a filter that has not run, the receiver asked
# for its navigation blocks on the pad and around the flight, at most three
# restarts in flight, and a restored flight's start waiting (3 s at most) for
# the barometer. Pure header in the flight_computer project; the crash-loop
# tests count boots with fc_rescue_policy.h as main.cpp does.
add_executable(test_raw_nav_policy test_raw_nav_policy.cpp)
target_include_directories(test_raw_nav_policy PRIVATE
    ${SHIM_DIR}
    ${LIB_DIR}/TR_RocketComputerTypes
    ${CMAKE_CURRENT_SOURCE_DIR}/../tinkerrocket-idf/projects/flight_computer/main
)
target_link_libraries(test_raw_nav_policy GTest::gtest_main)
gtest_discover_tests(test_raw_nav_policy)
