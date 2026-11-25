# tests_cpp/v9raw

Test registrations for the raw-GNSS navigation filter on the mosaic-G5 V8/V9
(branch fc/v9-raw-primary). tests_cpp/CMakeLists.txt includes every
`v9raw/*.cmake` here in sorted order, so each piece adds its own file instead
of editing the shared CMakeLists. A .cmake file declares its executables the
way the targets in tests_cpp/CMakeLists.txt do (LIB_DIR, SHIM_DIR,
GTest::gtest_main, gtest_discover_tests).

SBF test blocks come from tests_cpp/sbf_build.h. Committed fixtures are
synthetic; real captures stay outside the repository.
