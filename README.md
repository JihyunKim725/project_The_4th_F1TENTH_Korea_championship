# project_The_4th_F1TENTH_Korea_championship

This repository contains multiple ROS catkin workspaces and packages used for the F1TENTH project (localization, perception, mapping, VESC/driver, and simulators).

## Layout
- loc_ws/ — localization-related workspaces and packages  
  - [loc_ws/src/CMakeLists.txt](loc_ws/src/CMakeLists.txt)  
  - Localization package: [loc_ws/localization_ws/src/robot_localization/CMakeLists.txt](loc_ws/localization_ws/src/robot_localization/CMakeLists.txt)  
  - Tests example: [loc_ws/localization_ws/src/robot_localization/test/test_ros_robot_localization_listener.cpp](loc_ws/localization_ws/src/robot_localization/test/test_ros_robot_localization_listener.cpp)
- amcl examples: [loc_ws/amcl_ws/src/navigation/amcl/examples/f1tenth_localization.launch](loc_ws/amcl_ws/src/navigation/amcl/examples/f1tenth_localization.launch), [loc_ws/amcl_ws/src/navigation/amcl/examples/tf.launch](loc_ws/amcl_ws/src/navigation/amcl/examples/tf.launch)
- perception_ws/ — perception and opponent tracker  
  - Package manifest: [perception_ws/src/perception/package.xml](perception_ws/src/perception/package.xml)  
  - Example launch: [perception_ws/src/perception/opponent_tracker/launch/opponent_track.launch](perception_ws/src/perception/opponent_tracker/launch/opponent_track.launch)
- MAP_ws/ — map controller and simulator  
  - Controller README: [MAP_ws/src/MAP-Controller/README.md](MAP_ws/src/MAP-Controller/README.md)  
  - Simulator package: [MAP_ws/src/MAP-Controller/F110_ROS_Simulator/package.xml](MAP_ws/src/MAP-Controller/F110_ROS_Simulator/package.xml)
- vesc_ws/ — VESC driver and messages  
  - CI workflow example: [vesc_ws/src/vesc/.github/workflows/ros1-ci.yaml](vesc_ws/src/vesc/.github/workflows/ros1-ci.yaml)
- maps/ — stored map images and YAMLs (e.g. `maps/0625.pgm`, `maps/0625.yaml`)

## Supported ROS distros
- Tested with ROS Melodic / ROS Noetic (CI uses melodic; some package README suggests Noetic). Use the distro matching your system.

## Build (per workspace)
Build each catkin workspace separately:

```sh
# example for loc_ws
cd loc_ws
source /opt/ros/<distro>/setup.bash
catkin_make
source devel/setup.bash

# example for perception_ws
cd perception_ws
source /opt/ros/<distro>/setup.bash
catkin_make
source devel/setup.bash
```

Or use catkin_tools:
```sh
cd <workspace>
source /opt/ros/<distro>/setup.bash
catkin build
source devel/setup.bash
```

Replace `<distro>` with `melodic` or `noetic`.

# Run examples
- Localization + AMCL launch (example):
  -  [loc_ws/amcl_ws/src/navigation/amcl/examples/f1tenth_localization.launch](loc_ws/amcl_ws/src/navigation/amcl/examples/f1tenth_localization.launch)
- TF / EKF related launches referenced in examples:
  - [loc_ws/amcl_ws/src/navigation/amcl/examples/tf.launch](loc_ws/amcl_ws/src/navigation/amcl/examples/tf.launch)
- Opponent tracker:
  - [perception_ws/src/perception/opponent_tracker/launch/opponent_track.launch](perception_ws/src/perception/opponent_tracker/launch/opponent_track.launch)

Use roslaunch to run a launch file, for example:
```
roslaunch loc_ws amcl_localization.launch  # adjust path/name as appropriate
```
# Tests
- Robot localization package includes rostest/rostest_gtest entries. See [loc_ws/localization_ws/src/robot_localization/CMakeLists.txt](loc_ws/localization_ws/src/robot_localization/CMakeLists.txt) for test targets.
# Maps
- Prebuilt maps are in the `maps`/ directory (PGM + YAML pairs) for simulation/localization.
# Contributing
- Follow ROS package structure when adding packages.
- Keep package.xml and CMakeLists.txt updated for dependencies (see examples in `perception_ws` and `robot_localization`).
# Notes
- Some packages assume dependencies like `message_generation`, `dynamic_reconfigure`, `tf2`, `cv_bridge`, etc. Inspect package manifests: [perception_ws/src/perception/package.xml](perception_ws/src/perception/package.xml) and [loc_ws/localization_ws/src/robot_localization/package.xml](loc_ws/localization_ws/src/robot_localization/package.xml).
- For CI hints, see: [vesc_ws/src/vesc/.github/workflows/ros1-ci.yaml](vesc_ws/src/vesc/.github/workflows/ros1-ci.yaml)
  
License: check individual package LICENSE files (example: robot_localization installs LICENSE via its CMakeLists). See installation entries in [loc_ws/localization_ws/src/robot_localization/CMakeLists.txt](loc_ws/localization_ws/src/robot_localization/CMakeLists.txt).
