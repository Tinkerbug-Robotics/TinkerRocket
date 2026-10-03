cmake_minimum_required(VERSION 3.16)
project(gnss_receiver C CXX)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Warnings for our C. core/ is also built for the ESP32-P4 later, so it stays
# strictly portable.
set(GNSS_C_WARNINGS -Wall -Wextra -Wpedantic -Wshadow -Wstrict-prototypes
    -Wmissing-prototypes)

# No fused multiply-add contraction: the loops must give the same float results
# on the host and on the P4 (see README.md, "Number formats").
add_compile_options($<$<COMPILE_LANGUAGE:C>:-ffp-contract=off>)

add_subdirectory(core)
add_subdirectory(fpga/model)
add_subdirectory(host)

option(GNSS_BUILD_TESTS "Build the unit tests (fetches GoogleTest)" ON)
if(GNSS_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
