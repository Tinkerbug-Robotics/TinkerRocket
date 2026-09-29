cmake_minimum_required(VERSION 3.16)

# First image for the PX1105R board's ESP32-P4 (hardware/legacy/gnss-px1105r-18mm-highpower-ext-ant): bridges the
# receiver's UART to USB-Serial-JTAG and to the host UART on J4. See main/px1105r_p4_bridge.c.
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
idf_build_set_property(MINIMAL_BUILD ON)
project(px1105r_p4_bridge)
