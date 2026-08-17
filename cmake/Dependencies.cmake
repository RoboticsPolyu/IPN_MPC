find_package(Eigen3 REQUIRED NO_MODULE)
find_package(GTSAM 4.3 REQUIRED)
find_package(Pangolin 0.8 REQUIRED)
find_package(yaml-cpp REQUIRED)

if(IPN_MPC_BUILD_ROS)
  find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    mavros_msgs
    message_generation
    roscpp
    std_msgs
  )

  catkin_package(
    CATKIN_DEPENDS
      geometry_msgs
      mavros_msgs
      message_runtime
      roscpp
      std_msgs
  )
endif()

