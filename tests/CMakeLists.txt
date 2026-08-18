add_executable(gtsam_compatibility_test gtsam_compatibility_test.cpp)
target_link_libraries(gtsam_compatibility_test PRIVATE IPN_MPC::core)

add_test(NAME gtsam_compatibility COMMAND gtsam_compatibility_test)

add_executable(hard_constraint_qp_test hard_constraint_qp_test.cpp)
target_link_libraries(hard_constraint_qp_test PRIVATE IPN_MPC::core)

add_test(NAME hard_constraint_qp COMMAND hard_constraint_qp_test)

add_executable(terminal_dynamics_factor_test terminal_dynamics_factor_test.cpp)
target_link_libraries(terminal_dynamics_factor_test PRIVATE IPN_MPC::core)

add_test(NAME terminal_dynamics_factor COMMAND terminal_dynamics_factor_test)

add_executable(lqr_terminal_weight_test lqr_terminal_weight_test.cpp)
target_link_libraries(lqr_terminal_weight_test PRIVATE IPN_MPC::core)

add_test(NAME lqr_terminal_weight COMMAND lqr_terminal_weight_test)

add_executable(energy_factor_test energy_factor_test.cpp)
target_link_libraries(energy_factor_test PRIVATE IPN_MPC::core)

add_test(NAME energy_factor COMMAND energy_factor_test)

add_executable(imu_simulation_test imu_simulation_test.cpp)
target_link_libraries(imu_simulation_test PRIVATE IPN_MPC::core)

add_test(
  NAME imu_simulation
  COMMAND imu_simulation_test ${PROJECT_SOURCE_DIR}/config/quadrotor_thrust_gyro.yaml
)

if(TARGET terminal_acceleration_gyro_mpc)
  add_test(
    NAME terminal_acceleration_gyro_mpc
    COMMAND terminal_acceleration_gyro_mpc
            ${PROJECT_SOURCE_DIR}/config/terminal_acceleration_gyro_mpc.yaml
            --headless
  )
endif()

if(TARGET terminal_acceleration_gyro_setpoint_mpc)
  add_test(
    NAME terminal_acceleration_gyro_setpoint_mpc
    COMMAND terminal_acceleration_gyro_setpoint_mpc
            ${PROJECT_SOURCE_DIR}/config/terminal_acceleration_gyro_mpc.yaml
            --headless
  )
endif()
