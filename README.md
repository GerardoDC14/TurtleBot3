# TurtleBot3 Workspace

ROS 2 workspace for TurtleBot3 exploration, HAZMAT detection, and custom mechanism control.

## Project Layout

```text
.
|- src/                         # ROS 2 packages only
|  |- explore_cpp
|  |- turtlebot3_explore
|  |- turtlebot3_poi_navigation
|  |- laser_scan_adjuster
|  |- motor_position_controller
|  |- joint_state_publisher_custom
|  |- hazmat_marker
|  '- HAZMAT_Detection/         # Git submodule
|- maps/                        # Occupancy maps (.pgm/.yaml)
|- models/                      # Robot models (URDF)
|- tools/                       # Standalone scripts/utilities
|- docs/                        # Logs and diagrams
'- data/                        # Numeric datasets
```

## Prerequisites

- ROS 2 (recommended: Humble)
- `colcon`, `vcstool`, Python 3
- Navigation stack dependencies used by the packages (`nav2`, `slam_toolbox`, etc.)

## Clone and Setup

```bash
git clone --recurse-submodules https://github.com/<your-user>/TurtleBot3.git
cd TurtleBot3
```

If you already cloned without submodules:

```bash
git submodule update --init --recursive
```

## Build

```bash
colcon build --symlink-install
source install/setup.bash
```

## Typical Run Flow

1. Start robot bringup (on TurtleBot or simulation).
2. Start SLAM:

```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false
```

3. Start navigation:

```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=false use_amcl:=true
```

4. Start exploration:

```bash
ros2 launch turtlebot3_explore explore_launch.py
```

5. Start HAZMAT pipeline:

```bash
ros2 run camara camara_server
ros2 run camara camara_client
ros2 run hazmat_marker hazmat_marker
```

## Useful Components

- Exploration (Python): `turtlebot3_explore`
- Exploration (C++): `explore_cpp`
- POI navigation UI: `turtlebot3_poi_navigation`
- Scan adapter: `laser_scan_adjuster`
- Motor/joint utilities: `motor_position_controller`, `joint_state_publisher_custom`

## Notes

- Build artifacts (`build/`, `install/`, `log/`, `src/build/`, `src/install/`, `src/log/`) are intentionally excluded from version control.
- The HAZMAT camera package depends on the `HAZMAT_Detection` submodule and external model assets.
