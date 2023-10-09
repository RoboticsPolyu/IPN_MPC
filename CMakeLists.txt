cmake_minimum_required(VERSION 3.10)
project(MPC_FGO)

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

set(CMAKE_CXX_FLAGS "-std=c++14 -g -Wall")

if(APPLE)
    set(ENV{CC} "/usr/local/opt/llvm/bin/clang-8")
    set(ENV{CXX} "/usr/local/opt/llvm/bin/clang++")
    set(ENV{LDFLAGS} "-L/usr/local/opt/llvm/lib")
    set(ENV{CPPFLAGS} "-I/usr/local/opt/llvm/include")
endif()


set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -Wno-reorder" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -DNDEBUG -Wno-reorder -O2" CACHE STRING "" FORCE)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "/usr/local/cmake/lib/cmake")

# third party libs
# Eigen
find_package(Eigen3 REQUIRED NO_MODULE)
include_directories(${EIGEN3_INCLUDE_DIR})

# opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# glog
find_package(Glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})

# tbb
find_package(TBB COMPONENTS tbb tbbmalloc)

# gtsam
find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})

# include
include_directories(include)

# Armadillo
find_package(Armadillo REQUIRED)
include_directories(${ARMADILLO_INCLUDE_DIRS})

# Pangolin
find_package(Pangolin 0.8 REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

# Find OSQP library and headers
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)

# Find yaml-cpp
find_package(yaml-cpp)
include_directories(${YAML_CPP_INCLUDE_DIRS})

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
            include/gtsam_wrapper.h
            src/calibration/Calibration_factor.cpp
            src/calibration/Calibration_inertial.cpp
            src/control/SE3_controller.cpp
            src/dynamics/Dynamics_control_factor.cpp
            src/dynamics/Dynamics_factor.cpp
            src/env_sensors_sim/IMU.cpp
            src/env_sensors_sim/Landmarks.cpp
            src/env_sensors_sim/Lidar.cpp
            src/dynamics/Quadrotor_SO3.cpp
            src/trajectory_generator/Trajectory_generator.cpp)
            
target_link_libraries(quadrotor_dynamics PUBLIC gtsam gtsam_unstable ${Pangolin_LIBRARIES} ${YAML_CPP_LIBRARIES} PRIVATE osqp::osqp OsqpEigen::OsqpEigen )

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

add_executable(simple_circle_motion_test app/circle_trajectory_test.cpp)
target_link_libraries(simple_circle_motion_test gtsam quadrotor_dynamics ${Pangolin_LIBRARIES})

add_executable(Dynamics_factor_test app/Dynamics_factor_test.cpp)
target_link_libraries(Dynamics_factor_test gtsam quadrotor_dynamics ${Pangolin_LIBRARIES})

add_executable(Joint_Estimation_Control app/Joint_Estimation_Control.cpp)
target_link_libraries(Joint_Estimation_Control gtsam quadrotor_dynamics ${Pangolin_LIBRARIES} ${YAML_CPP_LIBRARIES})

add_executable(Joint_Estimation_Control_iSAM app/Joint_Estimation_Control_iSAM.cpp)
target_link_libraries(Joint_Estimation_Control_iSAM gtsam quadrotor_dynamics ${Pangolin_LIBRARIES} ${YAML_CPP_LIBRARIES})

add_executable(SW_Joint_Estimation_Control app/SW_Joint_Estimation_Control.cpp)
target_link_libraries(SW_Joint_Estimation_Control gtsam quadrotor_dynamics ${Pangolin_LIBRARIES} ${YAML_CPP_LIBRARIES})

add_executable(Joint_Positioning_Control_Test app/Joint_Positioning_Control_Test.cpp)
target_link_libraries(Joint_Positioning_Control_Test gtsam quadrotor_dynamics ${Pangolin_LIBRARIES} ${YAML_CPP_LIBRARIES})

# CALIBRATION
add_executable(Aerial_vehicle_calibration_BLACK app/Aerial_vehicle_calibration_BLACK.cpp)
target_link_libraries(Aerial_vehicle_calibration_BLACK gtsam quadrotor_dynamics ${Pangolin_LIBRARIES} ${YAML_CPP_LIBRARIES})

# add_executable(Aerial_vehicle_calibration_sim app/Aerial_vehicle_calibration_sim.cpp)
# target_link_libraries(Aerial_vehicle_calibration_sim gtsam quadrotor_dynamics ${Pangolin_LIBRARIES} ${YAML_CPP_LIBRARIES})

add_executable(Aerial_vehicle_calibration_NBEM app/Aerial_vehicle_calibration_NBEM.cpp)
target_link_libraries(Aerial_vehicle_calibration_NBEM quadrotor_dynamics ${Pangolin_LIBRARIES} ${YAML_CPP_LIBRARIES})

add_executable(Motor_calibration app/Motor_calibration.cpp)
target_link_libraries(Motor_calibration quadrotor_dynamics ${Pangolin_LIBRARIES} ${YAML_CPP_LIBRARIES})