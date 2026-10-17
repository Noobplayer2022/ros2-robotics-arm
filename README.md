# ROS 2 Robotic Arm and Conveyor Simulation

**A ROS 2 Lyrical Luth robotics project for manipulator modeling, motion planning, control, and pick-and-place simulation.**

Repository: [Noobplayer2022/ros2-robotics-arm](https://github.com/Noobplayer2022/ros2-robotics-arm)

## 1. Project Overview

This project integrates a multi-joint robotic arm, gripper components, conveyor simulation, robot visualization, motion planning, and control algorithms in a ROS 2 workspace.

The system is designed as a robotics development and simulation platform for studying manipulator control, trajectory tracking, object manipulation, and controller performance.

The project combines robot modeling, simulation, motion planning, and control software in a modular package structure. It also includes recorded benchmark results for comparing selected control strategies.

### Main Objectives

- Model and visualize a multi-degree-of-freedom robotic manipulator.
- Integrate gripper and conveyor-related simulation components.
- Configure robot visualization and motion planning using RViz 2 and MoveIt 2.
- Develop and evaluate PID, Model Predictive Control (MPC), and Sliding Mode Control (SMC).
- Explore Particle Swarm Optimization (PSO) for controller parameter tuning.
- Provide a foundation for pick-and-place tasks and perception-based manipulation.
- Support reproducible simulation experiments and controller performance analysis.

## 2. Technology Stack

| Component | Technology |
|---|---|
| Operating system | Ubuntu 26.04 LTS |
| Robotics middleware | ROS 2 Lyrical Luth |
| Build system | colcon and CMake |
| Robot description | URDF |
| Robot geometry | STL meshes |
| Visualization | RViz 2 |
| Motion planning | MoveIt 2 |
| Simulation | Gazebo with compatible ROS integration |
| Control | Python and C++ |
| Configuration | YAML and XML |
| Performance analysis | CSV results and plots |

The exact versions of Gazebo, MoveIt 2, Python dependencies, and simulation plugins must be compatible with the installed ROS 2 Lyrical environment.

## 3. Project Features

### Robotic Arm Modeling

The `combined_pkg` package contains the main robot description and supporting geometry.

Relevant resources include:

- `urdf/combine.urdf`
- `meshes/`
- `launch/display.launch.py`
- `launch/gazebo.launch.py`
- `config/`

The robot description defines the model structure and its links and joints. STL files provide the associated geometry.

### Gripper Integration

The workspace contains a dedicated `magnet_gripper_gz` package with a Gazebo-oriented C++ plugin component.

This package provides a foundation for integrating gripper-related simulation behavior. Its operation depends on the plugin interface, Gazebo version, and system configuration.

### Conveyor and Object Simulation

The project includes a separate `conveyor_sim_ros2` repository and a simulated cube model:

`combined_pkg/models/pick_cube/model.sdf`

These components support conveyor-related and object-manipulation experiments. The exact conveyor behavior depends on the source code and configuration of the conveyor package.

### Motion Planning

The `combined_pkg_moveit_config` package contains:

- SRDF robot semantic descriptions
- Kinematics configuration
- Joint limits
- MoveIt controller configuration
- RViz configuration
- Launch files for planning and visualization

These resources provide the configuration needed to integrate the robotic manipulator with MoveIt 2.

### Controller Development

The project includes scripts associated with:

- PID control
- Model Predictive Control (MPC)
- Sliding Mode Control (SMC)
- Particle Swarm Optimization (PSO)
- State-space modeling
- Motion control
- Perception
- Live plotting
- Controller benchmarking

The effectiveness of each method depends on the robot model, control interfaces, trajectory, parameter tuning, and simulation conditions.

## 4. Repository Structure

The expected workspace structure is:

```text
ros2-robotics-arm/
├── .gitignore
├── README.md
└── src/
    ├── combined_pkg/
    │   ├── benchmark_results/
    │   ├── config/
    │   ├── launch/
    │   ├── meshes/
    │   ├── models/
    │   ├── scripts/
    │   ├── urdf/
    │   ├── CMakeLists.txt
    │   └── package.xml
    │
    ├── combined_pkg_moveit_config/
    │   ├── config/
    │   ├── launch/
    │   ├── CMakeLists.txt
    │   └── package.xml
    │
    ├── magnet_gripper_gz/
    │   ├── include/
    │   ├── src/
    │   ├── CMakeLists.txt
    │   └── package.xml
    │
    ├── conveyor_sim_ros2/
    │
    └── my_first_package/
        ├── my_first_package/
        ├── test/
        ├── package.xml
        └── setup.py
```

The `build/`, `install/`, and `log/` directories are generated locally by colcon and should generally not be committed.

## 5. Requirements

Before building the project, ensure that the following are installed and configured:

- Ubuntu 26.04 LTS
- ROS 2 Lyrical Luth
- `colcon` build tools
- `rosdep`
- CMake and a C++ compiler
- Python 3
- Compatible Gazebo integration packages
- MoveIt 2 and RViz 2
- Additional dependencies declared by each package

Official ROS 2 documentation:

https://docs.ros.org/

The package dependencies and simulator plugins must be checked against the actual Lyrical installation.

## 6. Installation

### Step 1: Verify ROS 2 Lyrical

Open a terminal and run:

```bash
source /opt/ros/lyrical/setup.bash

echo $ROS_DISTRO
```

Expected output:

```text
lyrical
```

If the command fails, verify that ROS 2 Lyrical is installed in `/opt/ros/lyrical`.

### Step 2: Install Build Tools

```bash
sudo apt update

sudo apt install -y \
  git \
  build-essential \
  cmake \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool
```

Initialize rosdep if necessary:

```bash
sudo rosdep init
rosdep update
```

If rosdep has already been initialized, do not repeat `sudo rosdep init`.

### Step 3: Clone the Repository

Clone the repository into a workspace directory:

```bash
git clone \
  https://github.com/Noobplayer2022/ros2-robotics-arm.git \
  ~/ros2_ws
```

Move into the workspace:

```bash
cd ~/ros2_ws
```

Verify the structure:

```bash
ls
ls src
```

The `src` directory should contain the project packages.

### Step 4: Check the Conveyor Repository

The conveyor component is a separate Git repository. If it is configured as a submodule, initialize it with:

```bash
cd ~/ros2_ws
git submodule update --init --recursive
```

This command requires a valid `.gitmodules` file in the parent repository.

If the conveyor package is not configured as a submodule, clone its repository separately using its actual Git URL into `~/ros2_ws/src/`.

Do not assume that a nested Git repository's source files are included in the parent repository.

### Step 5: Install Package Dependencies

Source ROS 2:

```bash
source /opt/ros/lyrical/setup.bash
```

Install dependencies declared by packages in the workspace:

```bash
cd ~/ros2_ws

rosdep install \
  --from-paths src \
  --ignore-src \
  -r -y
```

If a dependency cannot be resolved, check its package name, its availability for Lyrical, and whether it must be built from source.

## 7. Build the Workspace

Build the packages:

```bash
cd ~/ros2_ws

source /opt/ros/lyrical/setup.bash

colcon build --symlink-install
```

After a successful build, source the workspace:

```bash
source ~/ros2_ws/install/setup.bash
```

Verify that ROS 2 can discover the packages:

```bash
ros2 pkg list | grep -E \
  'combined_pkg|magnet_gripper_gz|my_first_package'
```

If the build fails, resolve the first compilation or dependency error before proceeding.

### Rebuild a Single Package

To rebuild the main robot package:

```bash
cd ~/ros2_ws

source /opt/ros/lyrical/setup.bash

colcon build \
  --symlink-install \
  --packages-select combined_pkg
```

Then source the workspace again:

```bash
source install/setup.bash
```

## 8. Running the Project

Run the following commands in terminals where ROS 2 Lyrical and the workspace have been sourced.

### 8.1 Robot Visualization

The main package contains `display.launch.py`.

```bash
source /opt/ros/lyrical/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch combined_pkg display.launch.py
```

This is the intended command if the launch file is installed under the `combined_pkg` package.

The project also contains RViz-related launch files. Check the installed launch filenames if a command cannot locate its launch file.

### 8.2 Gazebo Simulation

The repository contains `gazebo.launch.py`.

```bash
source /opt/ros/lyrical/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch combined_pkg gazebo.launch.py
```

This command assumes the launch file and its required dependencies are installed correctly.

The Gazebo version and plugin interfaces must match those expected by the project.

### 8.3 MoveIt 2

The MoveIt configuration package contains `demo.launch.py`.

```bash
source /opt/ros/lyrical/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch combined_pkg_moveit_config demo.launch.py
```

The demo launch file is intended to start the configured planning environment.

Verify that the robot description, semantic description, kinematics settings, joint limits, and controller configuration are consistent before using the planning environment.

### 8.4 Inspect Running Nodes

Open another terminal:

```bash
source /opt/ros/lyrical/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 node list
```

Inspect available topics:

```bash
ros2 topic list
```

Inspect the information for a specific node:

```bash
ros2 node info /node_name
```

Replace `/node_name` with the name of an actual running node.

These commands help identify the joint-state publisher, controllers, simulation interfaces, and other active ROS 2 components.

## 9. Control Algorithms

The main control scripts are located in:

```text
src/combined_pkg/scripts/
```

### PID Control

PID control combines proportional, integral, and derivative feedback to reduce tracking error.

Its performance depends on gain selection, the robot's dynamics, the reference trajectory, and actuator constraints.

### Model Predictive Control

MPC uses a model to predict future system behavior and calculate control actions over a finite prediction horizon.

The project includes an MPC controller script and state-space modeling utilities.

### Sliding Mode Control

SMC is a robust control approach based on a designed sliding surface and a control law that drives the system toward the desired sliding dynamics.

Practical behavior depends on the selected sliding surface, control gains, model accuracy, and implementation.

### Particle Swarm Optimization

PSO is a population-based optimization method that can be used to tune selected controller parameters.

The project contains a PSO optimizer script. Its objective function, parameter bounds, and integration with the controllers should be checked in the source code before executing experiments.

### Relevant Scripts

| Script | Intended purpose |
|---|---|
| `arm_state_space.py` | State-space model calculations |
| `benchmark_controllers.py` | Controller benchmarking |
| `pid_controller_node.py` | PID controller implementation |
| `mpc_controller_node.py` | MPC controller implementation |
| `smc_controller_node.py` | SMC controller implementation |
| `pso_optimizer.py` | PSO optimization |
| `motion_control.py` | Motion-control functionality |
| `perception.py` | Perception-related functionality |
| `arm_plots.py` | Plotting and analysis |
| `live_plot.py` | Live visualization |
| `control_panel.py` | Control panel functionality |
| `cube_spawner.py` | Cube spawning functionality |
| `magnet_gripper_node.py` | Gripper-related node |

These descriptions indicate the apparent purpose of each filename. Exact execution requirements and runtime behavior depend on the implementation.

## 10. Controller Benchmarking

The repository includes recorded benchmark data and plots under:

```text
src/combined_pkg/benchmark_results/
```

The recorded experiment directory includes:

```text
20261005_150958/
```

Available results include controller trajectories, error plots, CSV measurements, and summary figures.

The recorded comparisons cover PID, MPC, and SMC, with PSO enabled and disabled.

To inspect the summary:

```bash
cat ~/ros2_ws/src/combined_pkg/benchmark_results/20261005_150958/summary.csv
```

To inspect the benchmarking implementation:

```bash
less ~/ros2_ws/src/combined_pkg/scripts/benchmark_controllers.py
```

The benchmark script should be reviewed to identify its supported arguments, input data, simulation requirements, and output paths before rerunning an experiment.

Existing results should not be treated as proof of performance under different trajectories, models, or simulation conditions without repeating the experiments.

## 11. Creating a New ROS 2 Package

ROS 2 packages should be created inside the workspace's `src/` directory.

### Create a Python Package

```bash
cd ~/ros2_ws/src

ros2 pkg create my_control_pkg \
  --build-type ament_python \
  --dependencies rclpy std_msgs
```

Create a Python node inside the generated Python module directory and register its executable in `setup.py`.

Build the package:

```bash
cd ~/ros2_ws

source /opt/ros/lyrical/setup.bash

colcon build \
  --symlink-install \
  --packages-select my_control_pkg
```

Source the workspace:

```bash
source install/setup.bash
```

Run the node using its registered executable name:

```bash
ros2 run my_control_pkg my_node
```

The executable must be registered correctly in `setup.py`.

### Create a C++ Package

```bash
cd ~/ros2_ws/src

ros2 pkg create my_cpp_control_pkg \
  --build-type ament_cmake \
  --dependencies rclcpp std_msgs
```

Implement the C++ node in the generated source directory and configure the target and installation rules in `CMakeLists.txt`.

Build the package:

```bash
cd ~/ros2_ws

source /opt/ros/lyrical/setup.bash

colcon build \
  --symlink-install \
  --packages-select my_cpp_control_pkg
```

Source and run the registered executable:

```bash
source install/setup.bash

ros2 run my_cpp_control_pkg my_node
```

### Add an Existing Package

Place the package directory inside `~/ros2_ws/src/`.

Check that it contains valid package metadata and the appropriate build files, such as `package.xml`, `CMakeLists.txt`, or `setup.py`.

Then install dependencies and rebuild:

```bash
cd ~/ros2_ws

rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install

source install/setup.bash
```

## 12. Development Workflow

A typical development cycle is:

1. Modify the robot description, meshes, control scripts, or configuration.
2. Rebuild the affected package.
3. Source the workspace.
4. Launch visualization, simulation, or planning.
5. Inspect topics, nodes, joint states, and controller behavior.
6. Record experiment data and evaluate performance.
7. Commit verified changes to Git.

To rebuild the workspace from scratch:

```bash
cd ~/ros2_ws

rm -rf build install log

source /opt/ros/lyrical/setup.bash

colcon build --symlink-install

source install/setup.bash
```

This removes generated build artifacts, not the source files under `src/`.

## 13. Troubleshooting

### ROS_DISTRO Is Not Lyrical

Check the sourced environment:

```bash
source /opt/ros/lyrical/setup.bash

echo $ROS_DISTRO
```

Expected output:

```text
lyrical
```

### Package Not Found

```bash
source /opt/ros/lyrical/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 pkg list | grep combined_pkg
```

If the package is missing, check the build output and package metadata.

### Launch File Not Found

Inspect the installed launch files:

```bash
find ~/ros2_ws/install \
  -path '*/share/*/launch/*' \
  -type f
```

Verify that the launch file is installed by the package's build configuration.

### Gazebo Plugin Errors

Check Gazebo compatibility, plugin libraries, model paths, and environment configuration.

Plugins built for a different Gazebo generation may not load correctly.

### Controller or Joint Errors

Verify joint names, controller interfaces, joint-state publication, and the controller YAML configuration.

Ensure that the configured interfaces match the simulated robot and that the expected controller components are running.

### Conveyor Package Missing

Check whether the conveyor repository was included in the parent repository or configured as a submodule.

A nested Git repository recorded as a Git link does not automatically include its source files in the parent checkout.

## 14. Future Development

Potential extensions include:

- Perception-driven pick-and-place tasks
- Improved conveyor-object tracking
- Trajectory tracking and controller comparison
- Automated parameter optimization
- More reproducible benchmarking
- Additional simulation tests
- Integration with physical robot hardware

These are possible development directions, not claims that every feature is already implemented.

## 15. License

The repository should specify the license that applies to its source code and assets.

Check the licenses of third-party models, meshes, plugins, and dependencies before redistribution.

## Author

**Ali Goodarzvand Chegini**

GitHub: [Noobplayer2022](https://github.com/Noobplayer2022)

Project repository: [ros2-robotics-arm](https://github.com/Noobplayer2022/ros2-robotics-arm)
