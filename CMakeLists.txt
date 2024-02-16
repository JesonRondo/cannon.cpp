# Set the minimum version of CMake that can be used
# To find the cmake version run
# $ cmake --version
cmake_minimum_required(VERSION 3.13)

# Set the project name
project (cannon)

# ---- Add source files ----

# Note: globbing sources is considered bad practice as CMake's generators may not detect new files
# automatically. Keep that in mind when changing files, or explicitly mention them here.
# file(GLOB_RECURSE headers CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/include/*.h")
# file(GLOB_RECURSE sources CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/source/*.cpp")
set(build_include_dirs
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
  "${CMAKE_CURRENT_SOURCE_DIR}")
include_directories(${build_include_dirs})

# Set compiler options, remove symbol tables and enable optimization level 2
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -s -O2")

set(CPP_FILES
  source/math/Mat3.cpp
  source/math/Vec3.cpp
  source/math/Quaternion.cpp
  source/utils/EventTarget.cpp
  source/utils/Pool.cpp
  source/utils/Vec3Pool.cpp
  source/shapes/Shape.cpp
  source/shapes/Sphere.cpp
  source/shapes/ConvexPolyhedron.cpp
  source/shapes/Box.cpp
  source/shapes/Plane.cpp
)

# My library, add anthor file modify here
add_library(cannon ${CPP_FILES})

# library with empty main function, for calc size
add_executable(cannon_lib
  source/Cannon.cpp
  ${CPP_FILES}
)

# GoogleTest requires at least C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

# unit test
add_executable(cannon_test
  test/vec3_test.cc
  test/mat3_test.cc
  test/quaternion_test.cc
  test/sphere_test.cc
  test/box_test.cc
)
target_link_libraries(cannon_test GTest::gtest_main cannon)

include(GoogleTest)
gtest_discover_tests(cannon_test)
