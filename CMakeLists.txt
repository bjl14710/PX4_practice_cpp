cmake_minimum_required(VERSION 3.20)
project(px4_practice_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

option(PX4P_BUILD_TESTS "Build the unit tests" ON)
option(PX4P_WERROR "Treat warnings as errors" OFF)

# Warnings shared by every target. -Wdouble-promotion catches accidental
# double math (e.g. writing 0.5 instead of 0.5f), which is slow on a
# flight controller with only a single-precision FPU. PX4 enables it too.
add_library(px4p_warnings INTERFACE)
target_compile_options(px4p_warnings INTERFACE
  -Wall -Wextra -Wpedantic -Wshadow -Wdouble-promotion -Wnon-virtual-dtor
  $<$<BOOL:${PX4P_WERROR}>:-Werror>)

add_library(px4p STATIC
  src/platform/hrt.cpp
  src/platform/work_queue.cpp
  src/uorb/uorb.cpp
  src/sim/quadrotor_sim.cpp
  src/sim/simulator_bridge.cpp
  src/modules/sensors/sensors.cpp
  src/modules/attitude_estimator/attitude_estimator.cpp
  src/modules/mc_att_control/mc_att_control.cpp
  src/modules/mc_rate_control/mc_rate_control.cpp
  src/modules/control_allocator/control_allocator.cpp
  src/modules/commander/commander.cpp
  src/modules/flight_script/flight_script.cpp
  src/modules/logger/logger.cpp
  src/flight_stack.cpp
)
target_include_directories(px4p PUBLIC src)
target_link_libraries(px4p PRIVATE px4p_warnings)

add_executable(px4p_sitl src/main.cpp)
target_link_libraries(px4p_sitl PRIVATE px4p px4p_warnings)

if(PX4P_BUILD_TESTS)
  enable_testing()

  # Use an installed GoogleTest if there is one, otherwise fetch it.
  find_package(GTest QUIET)
  if(NOT GTest_FOUND)
    include(FetchContent)
    FetchContent_Declare(googletest
      GIT_REPOSITORY https://github.com/google/googletest.git
      GIT_TAG v1.15.2
      GIT_SHALLOW TRUE)
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
  endif()

  add_executable(px4p_tests
    tests/test_uorb.cpp
    tests/test_work_queue.cpp
    tests/test_mathlib.cpp
    tests/test_quadrotor_sim.cpp
    tests/test_attitude_estimator.cpp
    tests/test_controllers.cpp
    tests/test_control_allocator.cpp
    tests/test_commander.cpp
    tests/test_closed_loop.cpp
  )
  target_link_libraries(px4p_tests PRIVATE px4p px4p_warnings GTest::gtest_main)

  include(GoogleTest)
  gtest_discover_tests(px4p_tests)
endif()
