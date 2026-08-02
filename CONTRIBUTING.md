# Contributing

## Scope

Contributions should improve reproducibility, package quality, documentation or clearly isolated robot capabilities. Avoid combining unrelated autonomy, perception and mechanism changes in one commit.

## Before opening a change

1. Update submodules without modifying their history unintentionally.
2. Run the repository-health gate:

```bash
python3 tools/repository_health.py
```

3. On a ROS 2 Humble machine, resolve dependencies and build the affected packages:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select <affected-packages>
```

4. Verify that `build/`, `install/`, `log/`, bags, databases and local environment files are not committed.

## Package expectations

Each first-party ROS package should include:

- a non-placeholder description;
- a named maintainer using the project no-reply address;
- explicit runtime dependencies;
- valid `setup.py` or `CMakeLists.txt` installation rules;
- a documented executable entry point;
- no generated workspace artifacts.

## Commit style

Use concise conventional-style messages when practical:

```text
feat(exploration): ...
fix(mapping): ...
docs: ...
chore(ros): ...
```

## Hardware changes

State explicitly whether a change was validated in simulation, from recorded data or on physical hardware. Do not describe an untested hardware path as verified.
