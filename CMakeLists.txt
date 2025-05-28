cmake_minimum_required(VERSION 2.8)
project(Multi_Camera_Calibration)
find_package(OpenCV REQUIRED)
find_package(cpp_utils REQUIRED)

set(CMAKE_CXX_STANDARD 17)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)
# Instruct CMake to run uic automatically when needed.
set(CMAKE_AUTOUIC ON)

set(SOURCES 
src/data.cpp 
src/load_pattern.h
)

# This will create you executable
add_executable(Multi_Camera_Calibration src/main.cpp  ${SOURCES})
target_link_libraries(Multi_Camera_Calibration ${OpenCV_LIBS} cpp_utils)
