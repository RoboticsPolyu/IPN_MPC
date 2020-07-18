cmake_minimum_required(VERSION 2.8)
project(calib)

set(DEFAULT_BUILD_TYPE "Release")
if (NOT CMAKE_BUILD_TYPE)
    message(STATUS "Setting build type to '${DEFAULT_BUILD_TYPE}' as none was specified.")
    set(CMAKE_BUILD_TYPE "${DEFAULT_BUILD_TYPE}" CACHE
            STRING "Choose the type of build." FORCE)
    # Set the possible values of build type for cmake-gui
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
            "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif ()

set(CMAKE_CXX_FLAGS "-std=c++11 -g -Wall")

message(STATUS "set env var")
set(ENV{CC} "/usr/local/opt/llvm/bin/clang")
set(ENV{CXX} "/usr/local/opt/llvm/bin/clang++")
set(ENV{LDFLAGS} "-L/usr/local/opt/llvm/lib")
set(ENV{CPPFLAGS} "-I/usr/local/opt/llvm/include")

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -Wno-reorder" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -DNDEBUG -Wno-reorder -O2" CACHE STRING "" FORCE)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

option(BUILD_APPS "Build APPs for slam course" YES)
option(BUILD_TESTS "Build test for slam course" No)

# third party libs
# eigen
find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIR})

# opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# glog
find_package(Glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})

# tbb
find_package(TBB 4.4 COMPONENTS tbb tbbmalloc)

#gtsam
find_package(gtsam REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})
message("GTSAM INCLUDE = " ${GTSAM_INCLUDE_DIR})

#include
include_directories(include)

set(IMU_SOURCE
    src/wio_factor.cpp
    src/wio_integration.cpp)

add_library(imu_factor_lib ${IMU_SOURCE})

add_executable(wheel_ext_cal app/test_wheel_imu_factor.cpp)
target_link_libraries(wheel_ext_cal gtsam imu_factor_lib)

add_executable(wheel_imu_pim app/test_wheel_imu_pim.cpp)
target_link_libraries(wheel_imu_pim gtsam imu_factor_lib)

add_executable(wheel_imu_factor_error app/test_wheel_imu_error.cpp)
target_link_libraries(wheel_imu_factor_error gtsam imu_factor_lib)

add_executable(wheel_imu_isam app/wheel_imu_isam.cpp)
target_link_libraries(wheel_imu_isam gtsam gtsam_unstable imu_factor_lib)