# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/christianpedersen/esp/esp-idf-v6.0/components/bootloader/subproject"
  "/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/flight_computer/build_v8/bootloader"
  "/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/flight_computer/build_v8/bootloader-prefix"
  "/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/flight_computer/build_v8/bootloader-prefix/tmp"
  "/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/flight_computer/build_v8/bootloader-prefix/src/bootloader-stamp"
  "/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/flight_computer/build_v8/bootloader-prefix/src"
  "/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/flight_computer/build_v8/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/flight_computer/build_v8/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/christianpedersen/Documents/Hobbies/ModelRockets/Code/.claude/worktrees/oc-board-headers/tinkerrocket-idf/projects/flight_computer/build_v8/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
