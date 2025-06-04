cmake_minimum_required(VERSION 3.10)
project(IPN_MPC)

option(BUILD_SHARED_LIBS "Build shared gtsam library, instead of static" ON)
option(BUILD_HARDWARE    "Enable Hardware Support" OFF)
option(BUILD_WITH_ROS    "Enable Ros Support" OFF)


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


set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -fpermissive -Wno-reorder" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -fpermissive -DNDEBUG -Wno-reorder -O2" CACHE STRING "" FORCE)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "/usr/local/cmake/lib/cmake")

if(BUILD_WITH_ROS)
find_package(catkin REQUIRED COMPONENTS
  roscpp message_generation std_msgs geometry_msgs
)

add_message_files(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/msg" FILES IMU.msg INPUT.msg Rsm.msg)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)
include_directories(${catkin_INCLUDE_DIRS})
catkin_package(
    CATKIN_DEPENDS 
    roscpp 
    mavros_msgs
    message_runtime 
    std_msgs 
    geometry_msgs)
endif()

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
# find_package(Armadillo REQUIRED)
# include_directories(${ARMADILLO_INCLUDE_DIRS})

# Pangolin
find_package(Pangolin 0.8 REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

# Find OSQP library and headers
# find_package(osqp REQUIRED)
# find_package(OsqpEigen REQUIRED)

# Find yaml-cpp
find_package(yaml-cpp)
include_directories(${YAML_CPP_INCLUDE_DIRS})

# OpenCV
find_package(OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )


IF(UNIX)
ELSEIF(APPLE)    
ENDIF()


add_library(quadrotor_dynamics 
            include/gtsam_wrapper.h
            src/UI.cpp
            src/control/Minimum_snap_controller.cpp
            src/control/Energy_control_factor.cpp
            src/dynamics/Dynamics_control_factor.cpp
            src/dynamics/Dynamics_factor.cpp
            src/env_sensors_sim/IMU.cpp
            src/env_sensors_sim/Landmarks.cpp
            src/env_sensors_sim/Lidar.cpp
            src/env_sensors_sim/Obs.cpp
            src/dynamics/Quadrotor_SO3.cpp
            src/trajectory_generator/Trajectory_generator.cpp)

target_link_libraries(quadrotor_dynamics PUBLIC gtsam gtsam_unstable ${Pangolin_LIBRARIES} ${YAML_CPP_LIBRARIES})


# Simulation of JPCM

add_executable(simple_circle_motion_test app/circle_trajectory_test.cpp)
target_link_libraries(simple_circle_motion_test gtsam quadrotor_dynamics ${Pangolin_LIBRARIES})

add_executable(Joint_Estimation_Control app/Joint_Estimation_Control.cpp)
target_link_libraries(Joint_Estimation_Control gtsam quadrotor_dynamics ${Pangolin_LIBRARIES} ${YAML_CPP_LIBRARIES})

add_executable(hin_Joint_Estimation_Control app/hin_Joint_Estimation_Control.cpp)
target_link_libraries(hin_Joint_Estimation_Control gtsam quadrotor_dynamics ${Pangolin_LIBRARIES} ${YAML_CPP_LIBRARIES})

add_executable(Joint_Estimation_Control_iSAM app/Joint_Estimation_Control_iSAM.cpp)
target_link_libraries(Joint_Estimation_Control_iSAM gtsam quadrotor_dynamics ${Pangolin_LIBRARIES} ${YAML_CPP_LIBRARIES})

add_executable(SW_Joint_Estimation_Control app/SW_Joint_Estimation_Control.cpp)
target_link_libraries(SW_Joint_Estimation_Control gtsam quadrotor_dynamics ${Pangolin_LIBRARIES} ${YAML_CPP_LIBRARIES})

add_executable(Joint_Positioning_Control_Test app/Joint_Positioning_Control_Test.cpp)
target_link_libraries(Joint_Positioning_Control_Test gtsam quadrotor_dynamics ${Pangolin_LIBRARIES} ${YAML_CPP_LIBRARIES})

add_executable(JPCM_TGyro_version_Test app/JPCM_TGyro_version_Test.cpp)
target_link_libraries(JPCM_TGyro_version_Test gtsam quadrotor_dynamics ${Pangolin_LIBRARIES} ${YAML_CPP_LIBRARIES})

add_executable(JPCM_TGyro_OA_Test app/JPCM_TGyro_OA_Test.cpp)
target_link_libraries(JPCM_TGyro_OA_Test gtsam quadrotor_dynamics ${Pangolin_LIBRARIES} ${YAML_CPP_LIBRARIES})