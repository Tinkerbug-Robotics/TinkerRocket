# ---------- test_antenna_readback_policy ----------
# Which copy of the GNSS antenna offset the OC's config readback serves: the
# FC's report ("fc") only when the FC is up with a v4 report, no antenna
# command the OC staged is unconfirmed, and the FC remembers a setting of its
# own or the cache is Unknown; else the OC's cache ("oc"). And when a report
# settles an unconfirmed command. Pure header in the OC project.
add_executable(test_antenna_readback_policy test_antenna_readback_policy.cpp)
target_include_directories(test_antenna_readback_policy PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../tinkerrocket-idf/projects/out_computer/main
)
target_link_libraries(test_antenna_readback_policy GTest::gtest_main)
gtest_discover_tests(test_antenna_readback_policy)
