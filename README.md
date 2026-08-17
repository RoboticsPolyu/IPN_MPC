# IPN_MPC

IPN_MPC is a C++17 research implementation of factor-graph model predictive control for
quadrotor navigation. It combines state estimation, nonlinear dynamics, trajectory tracking,
control limits, and obstacle avoidance in a GTSAM factor graph.

The current codebase targets GTSAM 4.3 and uses Pangolin for interactive 3D visualization.

## Features

- Joint positioning and model predictive control with factor-graph optimization.
- Variable-horizon terminal rollout factors for state-to-state and direct set-point tracking.
- Quadrotor dynamics with thrust, body-rate commands, drag, and simulated measurements.
- Circular, figure-eight, and back-and-forth reference trajectories.
- Moving spherical and cylindrical obstacle simulation.
- Soft Control Barrier Function (CBF) obstacle factors.
- Optional experimental hard-inequality SQP using GTSAM's active-set QP solver.
- A discrete-time safety boundary as a final collision guard.
- Timestamped, severity-based application logging.
- Automated GTSAM compatibility and hard-constraint QP tests.

## Requirements

- CMake 3.16 or newer
- A C++17 compiler
- Eigen3
- GTSAM 4.3 or newer, including `gtsam_unstable`
- Pangolin 0.8 or newer
- yaml-cpp

ROS and hardware integration dependencies are optional and disabled by default.

## Build and test

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

Useful CMake options:

```text
IPN_MPC_BUILD_APPS       Build simulation applications (default: ON)
IPN_MPC_BUILD_TESTS      Build automated tests (default: ON)
IPN_MPC_BUILD_ROS        Build ROS integration (default: OFF)
IPN_MPC_BUILD_HARDWARE   Build hardware integration (default: OFF)
```

For example:

```bash
cmake -S . -B build \
  -DIPN_MPC_BUILD_ROS=ON \
  -DIPN_MPC_BUILD_HARDWARE=ON
```

## Running a simulation

Applications accept configuration paths as positional command-line arguments. If an argument is
omitted, the application uses its default path under `../config/`, so the defaults are intended to
be run from `build/`:

```bash
cd build
./jpcm_thrust_gyro_cbf
```

Each application logs the resolved configuration paths at startup. Explicit paths may be relative
to the current directory or absolute.

| Executable | Arguments and defaults |
| --- | --- |
| `circle_trajectory` | `[simulator_config=../config/quadrotor_thrust_gyro.yaml]` |
| `constrained_joint_estimation_control` | `[factor_graph_config=../config/factor_graph_hin.yaml] [quadrotor_config=../config/quadrotor.yaml] [simulator_config=../config/quadrotor_thrust_gyro.yaml]` |
| `joint_estimation_control` | `[factor_graph_config=../config/factor_graph.yaml] [quadrotor_config=../config/quadrotor.yaml] [simulator_config=../config/quadrotor_thrust_gyro.yaml]` |
| `joint_estimation_control_isam` | `[simulator_config=../config/quadrotor_thrust_gyro.yaml]` |
| `jpcm_thrust_gyro` | `[factor_graph_config=../config/factor_graph_thrust_gyro.yaml] [quadrotor_config=../config/quadrotor_thrust_gyro.yaml]` |
| `jpcm_thrust_gyro_cbf` | `[factor_graph_config=../config/factor_graph_thrust_gyro_cbf.yaml] [quadrotor_config=../config/quadrotor_thrust_gyro_cbf.yaml]` |
| `jpcm_thrust_gyro_wall` | `[factor_graph_config=../config/factor_graph_thrust_gyro.yaml] [quadrotor_config=../config/quadrotor_thrust_gyro.yaml]` |
| `sliding_window_joint_estimation_control` | `[factor_graph_config=../config/factor_graph.yaml] [quadrotor_config=../config/quadrotor.yaml] [simulator_config=../config/quadrotor_thrust_gyro.yaml]` |
| `terminal_acceleration_gyro_mpc` | `[mpc_config=../config/terminal_acceleration_gyro_mpc.yaml] [--headless]` |
| `terminal_acceleration_gyro_setpoint_mpc` | `[mpc_config=../config/terminal_acceleration_gyro_mpc.yaml] [--headless]` |

For example, select custom factor-graph and quadrotor configurations with:

```bash
./jpcm_thrust_gyro_cbf \
  ../config/factor_graph_thrust_gyro_cbf.yaml \
  ../config/quadrotor_thrust_gyro_cbf.yaml
```

The terminal-factor circle-tracking MPC can be run interactively or without visualization:

```bash
./terminal_acceleration_gyro_mpc ../config/terminal_acceleration_gyro_mpc.yaml
./terminal_acceleration_gyro_mpc ../config/terminal_acceleration_gyro_mpc.yaml --headless
./terminal_acceleration_gyro_setpoint_mpc ../config/terminal_acceleration_gyro_mpc.yaml
./terminal_acceleration_gyro_setpoint_mpc ../config/terminal_acceleration_gyro_mpc.yaml --headless
```

The `quadrotor_config` value inside the terminal MPC YAML is resolved relative to the MPC
configuration file. This keeps a copied configuration portable when it refers to another YAML file
in the same directory.

The Pangolin window displays the vehicle, historical trajectory, prediction horizon, reference
trajectory, coordinate frame, configured obstacles, and runtime/solver statistics. Closing the
window exits an active simulation. The terminal MPC visualization remains open after its configured
iterations finish so the final result can be inspected.

## Set-point terminal MPC factor

`TerminalAccelerationGyroMeasurementFactor` directly compares a fixed pose and velocity set point
with the state predicted from the current state and a prefix of the control horizon:

```text
x_hat_j = f(x_0, u_0, ..., u_{j-1})
e_j = [p_j_ref - p_hat_j,
       Log(R_hat_j^-1 R_j_ref),
       v_j_ref - v_hat_j]
cost_j = 0.5 e_j^T Q_j e_j
```

The factor graph variables are only `X(0)`, `V(0)`, and `U(0)` through `U(j-1)`. The set point is
stored as a factor measurement, so no future `X(j)` or `V(j)` variable is introduced. The set-point
MPC executable adds one factor for every prediction step from 1 through `horizon`.

The factor uses analytical chain-rule Jacobians. Translation and velocity derivatives use the
closed-form constant-acceleration rollout, while rotation derivatives propagate SO(3) exponential
map sensitivities through the remaining rotation-product suffix. It does not use numerical
differentiation.

With the default 20-step configuration on the development machine, replacing numerical
differentiation reduced mean solver time from 15.73 ms to 1.29 ms and P95 solver time from 16.30 ms
to 1.54 ms. Actual timing depends on the build type, processor, horizon, and optimizer settings.

## Logging

Project logs include a timestamp, severity, source file, and line number. The default level is
`info`. Set `IPN_MPC_LOG_LEVEL` to `debug`, `info`, `warning`, `error`, or `off`:

```bash
IPN_MPC_LOG_LEVEL=debug ./jpcm_thrust_gyro_cbf
```

GTSAM optimizer iteration output is disabled by default so application diagnostics remain
readable.

## Obstacle avoidance

The main CBF example uses:

```text
apps/jpcm_thrust_gyro_cbf.cpp
config/factor_graph_thrust_gyro_cbf.yaml
config/quadrotor_thrust_gyro_cbf.yaml
```

Soft CBF factors guide the optimized trajectory away from obstacles. The following parameters
control the safety behavior:

```yaml
point_obs_sigma: 0.05
cbf_alpha: 0.2
cbf_beta: 0.5
safe_d: 0.15
uav_size: 0.20
```

The effective required clearance is:

```text
obstacle radius + safe_d + uav_size / 2
```

### Experimental hard constraints

GTSAM's standard nonlinear optimizers do not directly support nonlinear inequality constraints.
This project includes an experimental sequential quadratic programming pass that linearizes each
obstacle boundary and solves hard inequalities with `gtsam_unstable::QPSolver`.

Enable it in `config/factor_graph_thrust_gyro_cbf.yaml`:

```yaml
use_hard_constraints: true
hard_constraint_iterations: 3
```

The active-set solver can be too slow for real-time visualization on some systems, so hard
constraints are disabled by default. Soft CBF factors and the runtime safety boundary remain
enabled.

## Project layout

```text
apps/                 Simulation and research executables
cmake/                Dependency and target definitions
config/               Runtime YAML configuration
docs/                 Mathematical notes and documentation images
include/ipn_mpc/      Public headers grouped by subsystem
src/                  Implementations grouped by subsystem
tests/                Compatibility and regression tests
tools/matlab/         MATLAB analysis tools
tools/python/         Python analysis and plotting tools
```

Public headers and implementations mirror the same subsystem organization, including `control`,
`dynamics`, `simulation`, `trajectory`, and `visualization`.

## Code style

The repository uses `.clang-format` with four-space indentation and a 100-column target. Format
changed C++ files before committing:

```bash
clang-format -i path/to/file.cpp path/to/file.h
```

## References

- [VeCBFPdFactor mathematical formulation](docs/vecbf_pd_factor_mathematical_formulation.pdf)
- [Simulation video](https://youtu.be/QBPwTr4mFy4)

## Authors

- Peiwen Yang
- Weisong Wen
- Shiyu Bai
- Li-Ta Hsu

Department of Aeronautical and Aviation Engineering, The Hong Kong Polytechnic University.

For questions, contact [peiwen1.yang@connect.polyu.hk](mailto:peiwen1.yang@connect.polyu.hk).
