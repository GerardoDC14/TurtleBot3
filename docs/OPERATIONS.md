# Build and Operations Guide

## 1. Supported baseline

The documented baseline is Ubuntu 22.04 with ROS 2 Humble. Other ROS 2 distributions may require dependency or launch-file adjustments.

## 2. Workspace preparation

```bash
source /opt/ros/humble/setup.bash

git submodule update --init --recursive
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

Additional non-ROS dependencies used by selected packages include OpenCV, Eigen, scikit-learn, PyYAML and Tk.

## 3. Build profiles

### Full workspace

```bash
colcon build --symlink-install
source install/setup.bash
```

### Autonomy packages only

```bash
colcon build --symlink-install --packages-select \
  explore_cpp \
  turtlebot3_explore \
  turtlebot3_poi_navigation \
  laser_scan_adjuster
```

### Perception annotation only

```bash
colcon build --symlink-install --packages-select hazmat_marker
```

### Mechanism utilities only

```bash
colcon build --symlink-install --packages-select \
  motor_position_controller \
  joint_state_publisher_custom
```

## 4. Mapping and navigation flow

Start the TurtleBot3 hardware or simulation first, then launch the mapping or localization stack appropriate for the mission.

```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false
```

For navigation against a known map:

```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=false use_amcl:=true
```

Do not run multiple nodes that publish the same map or localization transform unless the launch configuration explicitly supports it.

## 5. Exploration

Python implementation:

```bash
ros2 launch turtlebot3_explore explore_launch.py
```

C++ implementation:

```bash
ros2 run explore_cpp explore_node
```

Run only one exploration mission client at a time. Both may dispatch goals to Nav2.

## 6. Point-of-interest navigation

```bash
ros2 run turtlebot3_poi_navigation poi_manager
```

The POI manager is intended for operator-defined, repeatable navigation goals. Confirm that localization is stable before sending a saved destination.

## 7. Scan normalization

```bash
ros2 run laser_scan_adjuster adjust_scan
```

The adapter subscribes to `/scan`, normalizes the sample count and publishes `/adjusted_scan`. Consumers must be configured explicitly if they should use the adjusted topic.

## 8. HAZMAT annotation

Initialize the external detector according to its own README, then run the first-party marker package:

```bash
ros2 run hazmat_marker hazmat_marker
```

Do not treat a visualization marker as a safety-certified detection. The marker is mission information for operator review.

## 9. Custom mechanism utilities

```bash
ros2 run motor_position_controller motor_position_controller
ros2 run joint_state_publisher_custom joint_state_publisher
```

The motor-position utility publishes bounded references at 20 Hz. It is an operator command generator, not a closed-loop actuator controller. Verify downstream consumers and limits before connecting real hardware.

## 10. Repository checks

```bash
python3 tools/repository_health.py
```

This verifies metadata and Python syntax without requiring ROS 2. A successful repository-health check does not replace a `colcon build` or hardware test.
