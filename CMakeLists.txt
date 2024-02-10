# Set the minimum version of CMake that can be used
# To find the cmake version run
# $ cmake --version
cmake_minimum_required(VERSION 3.5)

# set compiler for Windows
set(CMAKE_CXX_COMPILER "C:/MinGW/bin/g++.exe")
set(CMAKE_C_COMPILER "C:/MinGW/bin/gcc.exe")

# Set the project name
project (cannon)

# ---- Add source files ----

# Note: globbing sources is considered bad practice as CMake's generators may not detect new files
# automatically. Keep that in mind when changing files, or explicitly mention them here.
# file(GLOB_RECURSE headers CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/include/*.h")
# file(GLOB_RECURSE sources CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/source/*.cpp")

# Add an executable
add_executable(cannon source/cannon.cpp)
