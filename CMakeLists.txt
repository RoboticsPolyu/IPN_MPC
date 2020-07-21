cmake_minimum_required(VERSION 3.10)
project(calib)

option(BUILD_SHARED_LIBS "Build shared gtsam library, instead of static" ON)

set(wio_factor_version 1.0)
set(wio_factor_soversion 1.0)

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

set(ENV{CC} "/usr/local/opt/llvm/bin/clang-8")
set(ENV{CXX} "/usr/local/opt/llvm/bin/clang++")
set(ENV{LDFLAGS} "-L/usr/local/opt/llvm/lib")
set(ENV{CPPFLAGS} "-I/usr/local/opt/llvm/include")

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -Wno-reorder" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -DNDEBUG -Wno-reorder -O2" CACHE STRING "" FORCE)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "/usr/local/cmake/lib/cmake")

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

# gtsam
find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})

# include
include_directories(include)

set(WIO_SOURCE
    src/wio_factor.cpp
    src/wio_integration.cpp)

IF(UNIX)
ELSEIF(APPLE)    
ENDIF()

message(STATUS "Building GTSAM - shared: ${BUILD_SHARED_LIBS}")
add_library(wio_factor_lib ${WIO_SOURCE})
target_include_directories(wio_factor_lib PUBLIC ${GTSAM_INCLUDE_DIR})
target_link_libraries(wio_factor_lib PUBLIC gtsam gtsam_unstable)
set_target_properties(wio_factor_lib PROPERTIES
    OUTPUT_NAME         wio_factor_lib
    CLEAN_DIRECT_OUTPUT 1
    VERSION             ${wio_factor_version}
    SOVERSION           ${wio_factor_soversion})

add_executable(wheel_imu_factor app/test_wheel_imu_factor.cpp)
target_link_libraries(wheel_imu_factor gtsam wio_factor_lib)

add_executable(wheel_imu_pim app/test_wheel_imu_pim.cpp)
target_link_libraries(wheel_imu_pim gtsam wio_factor_lib)

add_executable(wheel_imu_error app/test_wheel_imu_error.cpp)
target_link_libraries(wheel_imu_error gtsam wio_factor_lib)

add_executable(wheel_imu_isam app/wheel_imu_isam.cpp)
target_link_libraries(wheel_imu_isam gtsam gtsam_unstable wio_factor_lib)
