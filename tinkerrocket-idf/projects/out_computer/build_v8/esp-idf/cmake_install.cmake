# Install script for directory: /Users/christianpedersen/esp/esp-idf-v6.0

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "TRUE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/Users/christianpedersen/.espressif/tools/xtensa-esp-elf/esp-15.2.0_20251204/xtensa-esp-elf/bin/xtensa-esp32s3-elf-objdump")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/xtensa/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_stdio/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_hal_dma/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_hal_gpspi/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_hal_clock/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_hal_mspi/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_blockdev/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_hal_security/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/bootloader/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esptool_py/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/partition_table/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_app_format/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_bootloader_format/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/app_update/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_partition/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/efuse/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_security/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_driver_gpio/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_hal_uart/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_pm/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_mm/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_driver_dma/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/mbedtls/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_hal_timg/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_hal_wdt/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_hal_ana_conv/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_hal_rtc_timer/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/bootloader_support/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/spi_flash/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_usb_cdc_rom_console/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_system/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_common/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_rom/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/hal/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/log/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/heap/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/soc/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_hal_gpio/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_hal_usb/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_hal_pmu/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_hal_touch_sens/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_hal_i2s/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_hw_support/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/freertos/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_libc/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/pthread/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/cxx/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_timer/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_ringbuf/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_psram/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_driver_uart/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_driver_gptimer/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/app_trace/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_event/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/nvs_sec_provider/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/nvs_flash/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_phy/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_driver_usb_serial_jtag/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/vfs/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/lwip/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_netif_stack/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_netif/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/wpa_supplicant/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_coex/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_wifi/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_driver_spi/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_gdbstub/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/bt/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/unity/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/cmock/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/console/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_hal_i2c/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_hal_twai/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/driver/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/http_parser/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp-tls/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_adc/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_blockdev_util/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_hal_ana_cmpr/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_driver_ana_cmpr/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_driver_bitscrambler/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_hal_cam/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_driver_isp/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_driver_cam/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_driver_dac/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_driver_i2c/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_driver_i2s/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_driver_i3c/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_hal_jpeg/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_driver_jpeg/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_hal_ledc/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_driver_ledc/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_hal_mcpwm/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_driver_mcpwm/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_hal_parlio/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_driver_parlio/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_hal_pcnt/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_driver_pcnt/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_hal_ppa/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_driver_ppa/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_hal_rmt/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_driver_rmt/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_hal_sd/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/sdmmc/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_driver_sd_intf/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_driver_sdio/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_driver_sdm/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_driver_sdmmc/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_driver_sdspi/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_driver_touch_sens/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_driver_tsens/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_driver_twai/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_eth/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_hal_ieee802154/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_hal_lcd/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_hid/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/tcp_transport/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_http_client/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_http_server/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_https_ota/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_https_server/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_lcd/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/protobuf-c/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/protocomm/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_local_ctrl/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/esp_trace/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/espcoredump/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/wear_levelling/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/fatfs/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/idf_test/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/ieee802154/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/openthread/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/perfmon/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/rt/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/spiffs/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/ulp/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/TR_Compat/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/TR_NVS/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/TR_RocketComputerTypes/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/CRC/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/TR_I2C_Interface/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/TR_I2S_Stream/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/TR_LogToFlash/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/TR_FlightLog/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/RadioLib/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/TR_LoRa_Comms/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/TR_Sensor_Data_Converter/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/TR_Orientation/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/TR_Coordinates/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/TR_OTA/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/TR_BLE_To_APP/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/TR_INA230/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/main/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/TR_BMP585/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/TR_BQ27Z746/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/TR_PID/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/TR_ControlMixer/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/TR_GNSSReceiverUBlox_Serial/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/TR_GeoMag/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/TR_GpsInsEKF/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/TR_IIS2MDC/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/TR_IMU_Int_V2/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/TR_ISM6HG256/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/TR_KinematicChecks/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/TR_MAX17205G/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/TR_MMC5983MA/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/TR_MagCalibrator/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/TR_Sensor_Collector/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/TR_Sensor_Collector_Sim/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/TR_ServoControl_ledc_mult/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/espressif__dhara/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/out_computer/build_v8/esp-idf/spi_nand_flash/cmake_install.cmake")
endif()

