cmake_minimum_required(VERSION 2.8)
project(Multi_Camera_Calibration)
find_package(OpenCV REQUIRED)
find_package(cpp_utils REQUIRED)

set(CMAKE_CXX_STANDARD 17)

# set(CMAKE_BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE Debug)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)
# Instruct CMake to run uic automatically when needed.
set(CMAKE_AUTOUIC ON)

##################################################################################
# BEGIN: const header cam serial numbers
##################################################################################
set(CONFIG_DIR ${CMAKE_CURRENT_SOURCE_DIR}/cfg)
set(FLIR_CAMERA_SERIAL_NUMBERS $ENV{FLIR_CAMERA_SERIAL_NUMBERS})
set(FLIR_CAMERA_COUNT $ENV{FLIR_CAMERA_COUNT})
set(FLIR_TOP_CAM_SERIAL $ENV{FLIR_TOP_CAM_SERIAL})
string(REPLACE "," ";" SERIAL_NUMBERS ${FLIR_CAMERA_SERIAL_NUMBERS})

set(GENERATED_HEADER "${CMAKE_CURRENT_SOURCE_DIR}/src/hardware_constants.h")
file(WRITE ${GENERATED_HEADER} "// This file is auto-generated. Do not edit.\n")
file(APPEND ${GENERATED_HEADER} "#pragma once\n\n")
file(APPEND ${GENERATED_HEADER} "#include <array>\n#include <string_view>\n\n")
file(APPEND ${GENERATED_HEADER} "constexpr std::string_view CONFIG_DIR = \"${CONFIG_DIR}\";\n")
file(APPEND ${GENERATED_HEADER} "constexpr std::size_t GLOBAL_CONST_NCAMS = ${FLIR_CAMERA_COUNT};\n")
file(
  APPEND 
  ${GENERATED_HEADER} 
  "constexpr std::array<std::string_view, ${FLIR_CAMERA_COUNT}> GLOBAL_CONST_CAMERA_SERIAL_NUMBERS = {\n")

foreach(val IN LISTS SERIAL_NUMBERS)
    file(APPEND ${GENERATED_HEADER} "    \"${val}\",\n")
endforeach()

file(APPEND ${GENERATED_HEADER} "};\n")
file(APPEND ${GENERATED_HEADER} "constexpr std::string_view GLOBAL_CONST_TOP_CAM_SERIAL = \"${FLIR_TOP_CAM_SERIAL}\";\n")
##################################################################################
# END: const header cam serial numbers
##################################################################################


set(SOURCES 
  src/data.cpp 
  src/load_pattern.h
  src/config_parser.cpp
)

# This will create you executable
add_executable(Multi_Camera_Calibration src/main.cpp  ${SOURCES})
target_link_libraries(Multi_Camera_Calibration ${OpenCV_LIBS} cpp_utils)
