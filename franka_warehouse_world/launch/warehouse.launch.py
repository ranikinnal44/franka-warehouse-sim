# Copyright (c) 2026 Franka test workspace
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""Launch an fr3 mounted on a table, with a single box on the tabletop.

Adapted from ``franka_gazebo_bringup``'s
``gazebo_franka_arm_example_controller.launch.py``. The difference is that the
arm's ``link0`` is fixed to the world at the tabletop height (the ``xyz`` xacro
arg -> ``world_joint`` in ``fr3.urdf.xacro``) so the arm sits on the table and
the box in front of it falls inside the FR3's ~0.85 m reach. The table and box
are baked into each world SDF.

Select the world with ``world:=small|large`` (which box size), or override the
mount height explicitly with ``mount_height:=<metres>``.
"""

import os
import xml.dom.minidom

import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchContext, LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, OpaqueFunction,
                            RegisterEventHandler)
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# world key -> (sdf filename, arm mount height in metres).
# Height = tabletop surface, so the arm is mounted on the table. Must match the
# --table-height used to generate the corresponding world.
WORLDS = {
    'small': ('warehouse_boxes_200x300x400.sdf', 0.75),
    'large': ('warehouse_boxes_200x400x600.sdf', 0.75),
}


def get_robot_description(context: LaunchContext):
    robot_type = context.perform_substitution(LaunchConfiguration('robot_type'))
    load_gripper = context.perform_substitution(LaunchConfiguration('load_gripper'))
    franka_hand = context.perform_substitution(LaunchConfiguration('franka_hand'))
    mount_height = resolve_mount_height(context)

    franka_xacro_file = os.path.join(
        get_package_share_directory('franka_gazebo_bringup'),
        'urdf', 'franka_arm.gazebo.xacro')

    robot_description_config = xacro.process_file(
        franka_xacro_file,
        mappings={
            'robot_type': robot_type,
            'hand': load_gripper,
            'gazebo': 'true',
            'ee_id': franka_hand,
            'gazebo_effort': 'true',
            # Fix link0 to the world at the mount height (see fr3.urdf.xacro).
            'xyz': f'0 0 {mount_height}',
        })

    if not isinstance(robot_description_config, xml.dom.minidom.Document):
        raise RuntimeError(
            f'The given xacro file {franka_xacro_file} is not a valid xml format.')

    return [Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description_config.toxml()}],
    )]


def resolve_world_path(context: LaunchContext):
    world = context.perform_substitution(LaunchConfiguration('world'))
    world_file = WORLDS[world][0] if world in WORLDS else world
    world_path = os.path.join(
        get_package_share_directory('franka_warehouse_world'), 'worlds', world_file)
    if not os.path.isfile(world_path):
        raise RuntimeError(
            f"World '{world}' not found at {world_path}. "
            f"Use one of {list(WORLDS)} or a filename in the worlds/ directory.")
    return world_path


def resolve_mount_height(context: LaunchContext):
    override = context.perform_substitution(LaunchConfiguration('mount_height'))
    if override:
        return float(override)
    world = context.perform_substitution(LaunchConfiguration('world'))
    if world in WORLDS:
        return WORLDS[world][1]
    return 0.0  # unknown custom world -> mount on the floor


def load_controller(context: LaunchContext):
    controller = context.perform_substitution(LaunchConfiguration('controller'))
    return [Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            controller,
            '--controller-manager-timeout', '30',
        ],
        parameters=[PathJoinSubstitution([
            FindPackageShare('franka_gazebo_bringup'),
            'config', 'franka_gazebo_controllers.yaml'])],
        output='screen',
    )]


def launch_setup(context: LaunchContext):
    world_path = resolve_world_path(context)

    # Let Gazebo find franka_description meshes and the bringup assets.
    os.environ['GZ_SIM_RESOURCE_PATH'] = os.pathsep.join([
        os.path.dirname(get_package_share_directory('franka_description')),
        get_package_share_directory('franka_gazebo_bringup')])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': f'{world_path} -r'}.items(),
    )

    spawn = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-topic', '/robot_description'],
        output='screen',
    )

    rviz_file = os.path.join(get_package_share_directory('franka_description'),
                             'rviz', 'visualize_franka.rviz')
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['--display-config', rviz_file, '-f', 'world'],
        condition=IfCondition(LaunchConfiguration('rviz')))

    return [
        gazebo,
        *get_robot_description(context),
        spawn,
        rviz_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn,
                on_exit=[OpaqueFunction(function=load_controller)],
            )),
        RegisterEventHandler(
            OnShutdown(on_shutdown=[ExecuteProcess(
                cmd=['pkill', '-SIGINT', 'ruby'],
                name='gz_sim_graceful_shutdown')])),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='small',
            description="Warehouse world: 'small' (200x300x400 mm boxes), "
                        "'large' (200x400x600 mm boxes), or a .sdf filename."),
        DeclareLaunchArgument(
            'mount_height', default_value='',
            description='Arm mounting height in metres. Empty = use the '
                        'per-world default (tabletop height, 0.75).'),
        DeclareLaunchArgument(
            'robot_type', default_value='fr3',
            description='Available values: fr3, fp3 and fer'),
        DeclareLaunchArgument(
            'load_gripper', default_value='false',
            description='true/false for activating the gripper'),
        DeclareLaunchArgument(
            'franka_hand', default_value='franka_hand',
            description='Default value: franka_hand'),
        DeclareLaunchArgument(
            'controller', default_value='gravity_compensation_example_controller',
            description='Controller from franka_example_controllers to load.'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='true/false for visualizing the robot in rviz'),
        OpaqueFunction(function=launch_setup),
    ])
