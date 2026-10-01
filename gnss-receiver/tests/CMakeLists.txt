# Unit tests: GoogleTest via FetchContent, as tests_cpp/ does. Tests that need
# the rig's IQ files are not here; see py/emul_check.py.
include(FetchContent)
FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)
include(GoogleTest)

add_executable(gnss_tests
    test_acq.cpp
    test_cmd_tags.cpp
    test_core_basics.cpp
    test_corr_float.cpp
    test_corr_model.cpp
    test_decim.cpp
    test_eph_pvt.cpp
    test_lnav.cpp
    test_trk.cpp
    test_fe_emul.cpp
    test_if_plan.cpp
    test_iir.cpp
    test_ini.cpp
    test_mix.cpp
    test_quant.cpp
    test_resamp.cpp
    test_rng.cpp
)
target_link_libraries(gnss_tests PRIVATE gnss_host GTest::gtest_main)
gtest_discover_tests(gnss_tests)
