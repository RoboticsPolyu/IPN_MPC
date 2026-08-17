cmake_minimum_required(VERSION 3.16)

project(IPN_MPC VERSION 0.1.0 LANGUAGES C CXX)

option(IPN_MPC_BUILD_APPS "Build simulation applications" ON)
option(IPN_MPC_BUILD_HARDWARE "Build hardware integration" OFF)
option(IPN_MPC_BUILD_ROS "Build ROS integration" OFF)
option(IPN_MPC_BUILD_TESTS "Build automated tests" ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release RelWithDebInfo MinSizeRel)
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(cmake/Dependencies.cmake)
include(cmake/Targets.cmake)

if(IPN_MPC_BUILD_TESTS)
  include(CTest)
  add_subdirectory(tests)
endif()
