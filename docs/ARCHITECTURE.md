# System Architecture

## 1. Purpose

The workspace combines mobile-robot autonomy, spatial perception and a custom mechanism into one ROS 2 development environment. The architecture is deliberately modular: mapping, exploration, navigation, mission annotation and mechanism control can be developed independently and then composed for an integrated run.

## 2. Runtime domains

### 2.1 State estimation and mapping

Inputs:

- TurtleBot3 odometry;
- the TF tree;
- 2D LaserScan data;
- optional scan normalization through `laser_scan_adjuster`.

SLAM Toolbox consumes odometry, TF and laser data to maintain an occupancy grid. The resulting map is the shared world representation used by exploration, navigation and RViz.

### 2.2 Exploration and navigation

The repository includes two exploration implementations:

- `explore_cpp` for a compiled C++ path with ROS 2 actions and OpenCV/Eigen support;
- `turtlebot3_explore` for a Python path that is easier to inspect and iterate.

Both are mission-level clients. They inspect the current map, select candidate exploration goals and delegate collision-aware motion to Nav2. Nav2 remains responsible for planning, local control and recovery behavior.

`turtlebot3_poi_navigation` complements exploration with operator-defined named destinations. This separates repeatable semantic navigation from unknown-space exploration.

### 2.3 HAZMAT perception and map annotation

The camera detector is maintained as the external `HAZMAT_Detection` submodule. First-party ROS 2 integration turns detections into mission information through `hazmat_marker`, allowing hazardous-material observations to appear as spatial annotations in RViz alongside the occupancy map.

This division keeps model inference separate from map and visualization responsibilities.

### 2.4 Custom mechanism support

`motor_position_controller` publishes bounded position references for two mechanism axes. `joint_state_publisher_custom` publishes the associated state representation used by robot visualization and downstream tooling.

These packages are intentionally independent from the mobile-base autonomy stack so the mechanism can be tested without running SLAM or Nav2.

## 3. Package boundaries

| Package | Owns | Does not own |
|---|---|---|
| `laser_scan_adjuster` | Scan sample normalization and republishing | Mapping or localization |
| `explore_cpp` | C++ exploration mission logic | Low-level trajectory control |
| `turtlebot3_explore` | Python exploration mission logic | Costmaps and controller execution |
| `turtlebot3_poi_navigation` | Named goals and navigation requests | Map generation |
| `hazmat_marker` | Mission annotation in RViz | Neural-network inference |
| `motor_position_controller` | Operator position references | Hardware motor drivers |
| `joint_state_publisher_custom` | Joint-state representation | Position-command generation |

## 4. Data-flow contracts

```text
/scan ──> laser_scan_adjuster ──> /adjusted_scan
odom + TF + scan ──> SLAM Toolbox ──> occupancy map
occupancy map + TF ──> exploration node ──> Nav2 goal
named POI ──> POI manager ──> Nav2 goal
camera detection ──> hazmat_marker ──> RViz marker
operator input ──> motor position topics ──> custom joint-state visualization
```

Topic names outside the explicitly documented adapters may vary across TurtleBot3 and Nav2 launch configurations. The operational guide therefore treats launch files and source package parameters as the source of truth.

## 5. Design characteristics

- **Alternative implementations:** C++ and Python exploration paths allow algorithm and performance comparisons.
- **Separation of concerns:** mapping, planning, perception and mechanism support remain independently runnable.
- **Mission-level abstraction:** custom nodes delegate motion safety and trajectory execution to Nav2.
- **External-model isolation:** the detector is a submodule rather than copied into first-party package history.
- **Reproducibility assets:** maps, robot models, data and diagrams are stored outside generated ROS directories.

## 6. Integration limitations

- A complete build depends on ROS 2 Humble and system packages not installed by Python tooling alone.
- The HAZMAT model assets and camera runtime are governed by the external submodule.
- Some custom-mechanism utilities are operator-oriented prototypes and do not include a hardware feedback loop.
- Hardware validation requires the matching TurtleBot3, sensors and mechanism.
