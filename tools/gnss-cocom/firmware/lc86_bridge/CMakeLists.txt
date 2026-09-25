cmake_minimum_required(VERSION 3.16)

# Bench image for the Tinker-Beetle's flight computer: bridges the LC86G's UART
# to USB-Serial-JTAG for the COCOM rig. See main/lc86_bridge.c.
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
idf_build_set_property(MINIMAL_BUILD ON)
project(lc86_bridge)
