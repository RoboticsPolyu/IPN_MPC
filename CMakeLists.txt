cmake_minimum_required(VERSION 3.10)
project(calib)

option(BUILD_SHARED_LIBS "Build shared gtsam library, instead of static" ON)

set(wio_factor_version 1.0)
set(wio_factor_soversion 1.0)
set(abb_wrapper_version 1.0)
set(abb_wrapper_soversion 1.0)

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
find_package(Eigen3 REQUIRED NO_MODULE)
include_directories(${EIGEN3_INCLUDE_DIR})

# opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# # glog
# find_package(Glog REQUIRED)
# include_directories(${GLOG_INCLUDE_DIRS})

# tbb
find_package(TBB COMPONENTS tbb tbbmalloc)

# gtsam
find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})

# include
include_directories(include)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/ode)

# Armadillo
find_package(Armadillo REQUIRED)
include_directories(${ARMADILLO_INCLUDE_DIRS})

# Pangolin
find_package(Pangolin 0.8 REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

# Find OSQP library and headers
find_package(osqp REQUIRED)

# # Link the OSQP shared library
# target_link_libraries(yourTarget PRIVATE osqp::osqp)

message("-- Pangolin_INCLUDE_DIRS:" ${Pangolin_INCLUDE_DIR})
set(WIO_SOURCE
    src/wio/wio_factor.cpp
    src/wio/wio_integration.cpp)

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

add_library(quadrotor_dynamics 
            src/dynamics/Quadrotor_SO3.cpp
            src/dynamics/Dynamics_factor.cpp
            src/trajectory_generator/Trajectory_generator.cpp)
            
target_link_libraries(quadrotor_dynamics PUBLIC gtsam gtsam_unstable ${Pangolin_LIBRARIES} PRIVATE osqp::osqp)
## Declare a cpp executable
#add_executable(odom_visualization src/odom_visualization.cpp)
# add_executable(quadrotor_simulator_so3
#   src/quadrotor_simulator_so3.cpp)

# target_link_libraries(quadrotor_simulator_so3 
#    ${catkin_LIBRARIES}
#    quadrotor_dynamics
# )

add_executable(test_dynamics src/test_dynamics/test_dynamics.cpp)
target_link_libraries(test_dynamics gtsam quadrotor_dynamics ${Pangolin_LIBRARIES})

add_executable(wheel_imu_factor app/test_wheel_imu_factor.cpp)
target_link_libraries(wheel_imu_factor gtsam wio_factor_lib)

add_executable(wheel_imu_pim app/test_wheel_imu_pim.cpp)
target_link_libraries(wheel_imu_pim gtsam wio_factor_lib)

add_executable(wheel_imu_error app/test_wheel_imu_error.cpp)
target_link_libraries(wheel_imu_error gtsam wio_factor_lib)

add_executable(wheel_imu_isam app/wheel_imu_isam.cpp)
target_link_libraries(wheel_imu_isam gtsam gtsam_unstable wio_factor_lib)
