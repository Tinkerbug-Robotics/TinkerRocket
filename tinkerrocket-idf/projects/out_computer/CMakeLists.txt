cmake_minimum_required(VERSION 3.16)
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../components")
# out_computer doesn't use guidance — exclude both the real submodule and
# the stub so neither ends up in the component list.
set(EXCLUDE_COMPONENTS "TR_GuidancePN TR_GuidancePN_Stub")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(out_computer)
