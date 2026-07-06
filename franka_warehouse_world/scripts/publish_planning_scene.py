#!/usr/bin/env python3
# Copyright (c) 2026 Franka test workspace
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""Add the warehouse table and box to MoveIt's planning scene.

The Gazebo world SDF is invisible to MoveIt -- move_group only knows the robot
URDF plus whatever collision objects are published into its planning scene.
This node reads the same config/scene.yaml the Gazebo world is generated from
and publishes the table (top slab + 4 legs) and the box as CollisionObjects, so
MoveIt plans around them and RViz's MotionPlanning display shows them.

Objects are expressed relative to the arm base frame (default ``fr3_link0``),
where the tabletop surface is z = 0 -- so this works regardless of the physical
mount height used in Gazebo.

Usage:
    ros2 run franka_warehouse_world publish_planning_scene.py \
        --ros-args -p world:=small -p config:=/path/to/scene.yaml
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile

import yaml
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive


def _box(dx, dy, dz):
    p = SolidPrimitive()
    p.type = SolidPrimitive.BOX
    p.dimensions = [float(dx), float(dy), float(dz)]
    return p


def _pose(x, y, z):
    pose = Pose()
    pose.position.x = float(x)
    pose.position.y = float(y)
    pose.position.z = float(z)
    pose.orientation.w = 1.0
    return pose


class PlanningScenePublisher(Node):
    def __init__(self):
        super().__init__('warehouse_planning_scene')
        self.frame_id = self.declare_parameter('frame_id', 'fr3_link0').value
        world = self.declare_parameter('world', 'small').value
        config_path = self.declare_parameter('config', '').value
        if not config_path:
            raise RuntimeError('the "config" parameter (path to scene.yaml) is required')

        with open(config_path) as f:
            cfg = yaml.safe_load(f)
        self.table = cfg['table']
        if world not in cfg['worlds']:
            raise RuntimeError(f"world '{world}' not in {list(cfg['worlds'])}")
        self.box_cfg = cfg['worlds'][world]

        # Latch the scene so move_group receives it even if it subscribes late.
        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(PlanningScene, '/planning_scene', qos)
        # Re-publish a few times in case move_group starts after us.
        self.count = 0
        self.timer = self.create_timer(1.0, self._tick)

    def _collision_objects(self):
        t = self.table
        # Table: top slab (surface at z=0, so slab centred at -thickness/2) + 4 legs.
        top_z = -t['thickness'] / 2.0
        leg_h = t['height'] - t['thickness']
        leg_cz = -t['thickness'] - leg_h / 2.0
        hx = t['size_x'] / 2.0 - t['leg'] / 2.0
        hy = t['size_y'] / 2.0 - t['leg'] / 2.0

        table = CollisionObject()
        table.header.frame_id = self.frame_id
        table.id = 'table'
        table.operation = CollisionObject.ADD
        table.primitives.append(_box(t['size_x'], t['size_y'], t['thickness']))
        table.primitive_poses.append(_pose(t['center_x'], 0.0, top_z))
        for sx in (-1.0, 1.0):
            for sy in (-1.0, 1.0):
                table.primitives.append(_box(t['leg'], t['leg'], leg_h))
                table.primitive_poses.append(
                    _pose(t['center_x'] + sx * hx, sy * hy, leg_cz))

        # Box: rests on the tabletop (bottom at z=0 -> centre at dz/2).
        bx, by = self.box_cfg['box_xy']
        dx, dy, dz = self.box_cfg['box_size']
        box = CollisionObject()
        box.header.frame_id = self.frame_id
        box.id = 'box'
        box.operation = CollisionObject.ADD
        box.primitives.append(_box(dx, dy, dz))
        box.primitive_poses.append(_pose(bx, by, dz / 2.0))
        return [table, box]

    def _tick(self):
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = self._collision_objects()
        self.pub.publish(scene)
        self.count += 1
        if self.count == 1:
            self.get_logger().info(
                f'Published table + box to /planning_scene (frame {self.frame_id}).')
        if self.count >= 5:
            self.timer.cancel()


def main():
    rclpy.init()
    node = PlanningScenePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
