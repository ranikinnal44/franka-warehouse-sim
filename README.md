# franka-warehouse-sim

A Gazebo (Ignition Fortress) warehouse test environment for the Franka **fr3**,
built on top of `franka_ros2`. It provides:

- **`franka_warehouse_world`** — worlds with a table + mounted arm + a single
  box, launch files for the sim, and a MoveIt planning-scene publisher.
This is a **meta repo**: the Franka stack is included as git submodules (forks
of `franka_ros2` and `franka_description` carrying the sim-specific changes —
force/torque sensor state export in the Gazebo gravity-compensation system, and
rviz configs — plus unmodified upstream `libfranka` and `olvx`). One recursive
clone gives the whole workspace `src`.

## Setup

Requires ROS 2 Humble + `ros-dev-tools` (`rosdep`, `colcon`).

```bash
# clone the meta repo (with submodules) directly as your workspace src/
git clone --recurse-submodules \
    https://github.com/ranikinnal44/franka-warehouse-sim.git ~/franka_ws/src

cd ~/franka_ws
rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF
source install/setup.bash
```

> Already cloned without `--recurse-submodules`? Run
> `git submodule update --init --recursive` inside the repo.

> Two apt packages are only needed by `franka_mobile_sensors` (not the
> warehouse sim): `ros-humble-realsense2-description`,
> `ros-humble-sick-safetyscanners2`.

## Run

```bash
# Gazebo sim (small = 200x300x400 mm box, large = 200x400x600 mm)
ros2 launch franka_warehouse_world warehouse.launch.py world:=large

# With MoveIt (planning scene incl. the table + box). Terminal 1:
ros2 launch franka_warehouse_world warehouse.launch.py world:=large rviz:=false
# Terminal 2:
ros2 launch franka_warehouse_world moveit.launch.py world:=large
```

See [`franka_warehouse_world/README.md`](franka_warehouse_world/README.md) for
details on the worlds, the arm mounting, and the MoveIt integration.

## Contents / provenance

| Component | Tracked as | Source |
|-----------|-----------|--------|
| `franka_warehouse_world` | in-repo | this repo (Apache-2.0) |
| `franka_ros2` | submodule `@warehouse-sim` | fork `ranikinnal44/franka_ros2` (upstream `frankarobotics` v2.5.0) |
| `franka_description` | submodule `@warehouse-sim` | fork `ranikinnal44/franka_description` (upstream 2.8.0) |
| `libfranka` | submodule (pinned 0.20.4) | upstream `frankarobotics/libfranka` (unmodified) |
| `olvx_descriptions_module` | submodule (main) | upstream `olive-robotics` (unmodified) |
