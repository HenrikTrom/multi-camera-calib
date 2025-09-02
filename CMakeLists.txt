cmake_minimum_required(VERSION 2.8)
project(Multi_Camera_Calibration)
find_package(OpenCV REQUIRED)
find_package(cpp_utils REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_BUILD_TYPE Debug)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)
# Instruct CMake to run uic automatically when needed.
set(CMAKE_AUTOUIC ON)

set(CONFIG_DIR ${CMAKE_CURRENT_SOURCE_DIR}/cfg)
set(FLIR_CAMERA_COUNT $ENV{FLIR_CAMERA_COUNT})

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/config.h.in
  ${CMAKE_CURRENT_SOURCE_DIR}/src/config.h
)

set(SOURCES 
  src/data.cpp 
  src/load_pattern.h
  src/config_parser.cpp
)

# This will create you executable
add_executable(Multi_Camera_Calibration src/main.cpp  ${SOURCES})
target_link_libraries(Multi_Camera_Calibration ${OpenCV_LIBS} cpp_utils)
