# Copyright (c) 2026 Franka test workspace
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""Publish the warehouse table + box into MoveIt's planning scene.

Run this alongside move_group so MoveIt plans around (and RViz shows) the same
table and box that exist in the Gazebo world. Reads config/scene.yaml.

    ros2 launch franka_warehouse_world planning_scene.launch.py world:=large
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('franka_warehouse_world'), 'config', 'scene.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='small',
            description="Which box size to add: 'small' or 'large'."),
        DeclareLaunchArgument(
            'frame_id', default_value='fr3_link0',
            description='Base frame the objects are placed relative to '
                        '(tabletop = z 0).'),
        Node(
            package='franka_warehouse_world',
            executable='publish_planning_scene.py',
            name='warehouse_planning_scene',
            output='screen',
            parameters=[{
                'world': LaunchConfiguration('world'),
                'frame_id': LaunchConfiguration('frame_id'),
                'config': config,
            }],
        ),
    ])
