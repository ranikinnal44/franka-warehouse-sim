# franka_warehouse_world

Warehouse Gazebo (Ignition Fortress) test worlds for the Franka **fr3**. Each
world is an enclosed 12 × 10 × 4 m room containing a **1.2 × 1.2 m table with
the arm mounted on it** and a **single box** of the configured size resting on
the tabletop. The box sits at (0.45, −0.3) — on the −Y half — leaving the +Y
half of the tabletop clear as a place target after a pick.

All geometry comes from [`config/scene.yaml`](config/scene.yaml), the single
source of truth shared by the Gazebo world generator and the MoveIt scene
publisher.

| World (`world:=`) | SDF file | Box size (L×W×H) | Box centre reach |
|-------------------|----------|------------------|------------------|
| `small` (default) | `warehouse_boxes_200x300x400.sdf` | 0.2 × 0.3 × 0.4 m | 0.58 m |
| `large`           | `warehouse_boxes_200x400x600.sdf` | 0.2 × 0.4 × 0.6 m | 0.62 m |

The tabletop is at 0.75 m; the arm's `link0` is fixed to the world at that
height via the `xyz` xacro arg (`world_joint` in `fr3.urdf.xacro`), so it sits
on the table. Both the box and the place target are well inside the FR3's
~0.85 m reach.

The box is a **dynamic** rigid body (graspable) by default. Regenerate with
`--static-box` to pin it in place.

## MoveIt: "No Planning Scene Loaded" and the table/box

Two separate facts:

1. **The planning scene is owned by `move_group`.** If RViz's MotionPlanning
   display says *"No Planning Scene Loaded"*, it means no `move_group` is
   running (or RViz isn't connected to it). `warehouse.launch.py` only starts
   Gazebo + a basic RobotModel RViz — no `move_group`.
2. **The Gazebo world SDF is never shared with MoveIt.** `move_group` only
   knows the robot URDF plus collision objects published into its scene, so the
   table/box must be added explicitly.

`moveit.launch.py` handles both — it starts `move_group` + MoveIt RViz and adds
the table/box. Run it alongside the sim:

```bash
# terminal 1 — sim without its own RViz
ros2 launch franka_warehouse_world warehouse.launch.py world:=large rviz:=false

# terminal 2 — MoveIt (move_group + MotionPlanning RViz) + the scene objects
ros2 launch franka_warehouse_world moveit.launch.py world:=large
```

To keep MoveIt's TF consistent with the sim, `move_group` uses the **same
mounted Gazebo description** the sim publishes (rooted at the `world` link,
`world` → `fr3_link0` fixed at the tabletop height). The stock SRDF's
`virtual_joint` (child link `base`, which doesn't exist in that description) is
stripped so MoveIt uses `world` as its model frame — identical to the sim.

The table (top slab + 4 legs) and box are published as `CollisionObject`s
relative to `fr3_link0` (tabletop = z 0) by `publish_planning_scene.py`. To add
them to an already-running `move_group` without starting another one:

```bash
ros2 launch franka_warehouse_world planning_scene.launch.py world:=large
```

## Launch

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# 200×300×400 mm stack (default)
ros2 launch franka_warehouse_world warehouse.launch.py

# 200×400×600 mm stack
ros2 launch franka_warehouse_world warehouse.launch.py world:=large
```

Other pass-through arguments: `robot_type` (fr3/fp3/fer), `load_gripper`,
`controller`, `rviz`. The launch reuses
`franka_gazebo_bringup`'s `gazebo_franka_arm_example_controller.launch.py`,
overriding only its `gz_args` to load the chosen world.

## Regenerating / customising worlds

The worlds are produced by `scripts/generate_world.py` and committed:

Edit [`config/scene.yaml`](config/scene.yaml) (table size, box sizes/positions,
mount height), then regenerate both worlds from it:

```bash
# from the package source directory
python3 scripts/generate_world.py --config config/scene.yaml --world small \
    --output worlds/warehouse_boxes_200x300x400.sdf
python3 scripts/generate_world.py --config config/scene.yaml --world large \
    --output worlds/warehouse_boxes_200x400x600.sdf
```

Add `--static-box` to pin the box. Because the MoveIt publisher reads the same
`scene.yaml`, both stay in sync automatically. Rebuild the package afterwards so
the installed copies update.
