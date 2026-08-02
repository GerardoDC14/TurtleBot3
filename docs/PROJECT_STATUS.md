# Project Status and Provenance

## Classification

**Portfolio reference / historical integrated robotics workspace**

This repository contains code from an early formal robotics project. It is intentionally presented as a multi-package system rather than rewritten into a fictional modern product.

## Maturity matrix

| Area | Status | Notes |
|---|---|---|
| Repository organization | Maintained | Generated ROS artifacts are excluded and first-party packages are grouped under `src/` |
| Package metadata | Maintained | Maintainer, description and package-install metadata are normalized |
| Autonomous exploration | Prototype / research implementation | C++ and Python alternatives are retained |
| SLAM and Nav2 integration | Demonstrated workflow | Depends on the TurtleBot3 and ROS 2 runtime |
| HAZMAT integration | Demonstrated integration | Inference lives in an external submodule |
| POI navigation | Operator tool | Intended for named-goal workflows |
| Scan adapter | Utility | Fixed-size scan normalization with sensor-data QoS |
| Custom mechanism tools | Prototype | Command and visualization utilities, not safety-rated control |
| CI | Repository-level | Metadata and Python syntax checks; not a full hardware build |

## Preserved historical characteristics

- Alternative implementations are retained instead of collapsed into one package.
- Experiment maps, data and models remain part of the technical record.
- The external HAZMAT detector remains a submodule with independent history.
- Hardware-specific behavior is documented rather than generalized beyond available evidence.

## Known limitations

- The workspace has not been converted into a binary distribution or container image.
- Full dependency resolution requires a ROS 2 Humble machine.
- Hardware, camera and model availability affect end-to-end reproducibility.
- Custom mechanism utilities do not verify physical actuator state.

## Recommended future work

1. Add rosbag-based reproducibility fixtures for mapping and marker workflows.
2. Add launch tests for package composition.
3. Parameterize scan size and topic names in `laser_scan_adjuster`.
4. Add a common mission launch file that selects exactly one exploration implementation.
5. Record representative RViz and map outputs under `docs/assets/`.
