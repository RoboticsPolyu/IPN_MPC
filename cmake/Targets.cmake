set(IPN_MPC_CORE_SOURCES
  src/control/cbf_factor.cpp
  src/control/dynamics_control_factor.cpp
  src/control/dynamics_factor.cpp
  src/control/energy_control_factor.cpp
  src/control/lqr_terminal_weight.cpp
  src/dynamics/quadrotor_so3.cpp
  src/simulation/imu.cpp
  src/simulation/landmarks.cpp
  src/simulation/lidar.cpp
  src/trajectory/trajectory_generator.cpp
  src/visualization/ui.cpp
)

add_library(ipn_mpc_core ${IPN_MPC_CORE_SOURCES})
add_library(IPN_MPC::core ALIAS ipn_mpc_core)

target_compile_features(ipn_mpc_core PUBLIC cxx_std_17)
target_compile_options(ipn_mpc_core PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(ipn_mpc_core
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(ipn_mpc_core
  PUBLIC
    Eigen3::Eigen
    gtsam
    gtsam_unstable
    ${Pangolin_LIBRARIES}
    yaml-cpp
)
target_include_directories(ipn_mpc_core SYSTEM PUBLIC ${Pangolin_INCLUDE_DIRS})

if(IPN_MPC_BUILD_ROS)
  target_include_directories(ipn_mpc_core PUBLIC ${catkin_INCLUDE_DIRS})
  target_link_libraries(ipn_mpc_core PUBLIC ${catkin_LIBRARIES})
  add_dependencies(ipn_mpc_core ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
endif()

if(IPN_MPC_BUILD_HARDWARE)
  target_sources(ipn_mpc_core PRIVATE
    src/hardware/mavros_middleware.cpp
    src/hardware/uart.cpp
  )
endif()

if(IPN_MPC_BUILD_APPS)
  set(IPN_MPC_APPS
    circle_trajectory
    constrained_joint_estimation_control
    joint_estimation_control
    joint_estimation_control_isam
    jpcm_thrust_gyro
    jpcm_thrust_gyro_cbf
    jpcm_thrust_gyro_wall
    sliding_window_joint_estimation_control
    terminal_acceleration_gyro_mpc
    terminal_acceleration_gyro_setpoint_mpc
  )

  foreach(app IN LISTS IPN_MPC_APPS)
    add_executable(${app} apps/${app}.cpp)
    target_link_libraries(${app} PRIVATE IPN_MPC::core)
  endforeach()
endif()
