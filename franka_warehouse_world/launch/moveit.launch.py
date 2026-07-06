# Copyright (c) 2026 Franka test workspace
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""Bring up MoveIt (move_group + RViz) for the warehouse sim and add the scene.

Run this ALONGSIDE ``warehouse.launch.py`` (start that with ``rviz:=false``).
It fixes the "No Planning Scene Loaded" message by actually starting
``move_group`` -- the node that owns the planning scene -- and it uses the same
mounted Gazebo robot description as the sim so their TF trees agree. It then
publishes the table + box into the scene.

Frame handling: the Gazebo description is rooted at the ``world`` link
(``world`` -> ``fr3_link0`` fixed at the tabletop height), so the redundant
``virtual_joint`` is stripped from the SRDF and MoveIt uses ``world`` as its
model frame -- identical to the sim's TF.

    ros2 launch franka_warehouse_world warehouse.launch.py rviz:=false &
    ros2 launch franka_warehouse_world moveit.launch.py world:=small
"""

import os
import re

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_yaml(package_name, file_path):
    path = os.path.join(get_package_share_directory(package_name), file_path)
    try:
        with open(path, 'r') as f:
            return yaml.safe_load(f)
    except EnvironmentError:
        return None


def build_descriptions(context: LaunchContext, load_gripper: str):
    scene_cfg = os.path.join(
        get_package_share_directory('franka_warehouse_world'), 'config', 'scene.yaml')
    with open(scene_cfg) as f:
        mount_height = yaml.safe_load(f)['table']['height']

    # robot_description: the SAME mounted Gazebo description the sim publishes,
    # so move_group's model matches the sim's TF exactly.
    gazebo_xacro = os.path.join(
        get_package_share_directory('franka_gazebo_bringup'),
        'urdf', 'franka_arm.gazebo.xacro')
    urdf = xacro.process_file(gazebo_xacro, mappings={
        'robot_type': 'fr3',
        'hand': load_gripper,
        'gazebo': 'true',
        'ee_id': 'franka_hand',
        'gazebo_effort': 'true',
        'xyz': f'0 0 {mount_height}',
    }).toxml()

    # SRDF: strip the virtual_joint (its child link 'base' does not exist in the
    # world-rooted Gazebo description; 'world' is already the URDF root).
    srdf_xacro = os.path.join(
        get_package_share_directory('franka_description'),
        'robots', 'fr3', 'fr3.srdf.xacro')
    srdf = xacro.process_file(srdf_xacro, mappings={
        'hand': load_gripper, 'ee_id': 'franka_hand'}).toxml()
    srdf = re.sub(r'<virtual_joint\b[^>]*/>', '', srdf)

    return {'robot_description': urdf}, {'robot_description_semantic': srdf}


def launch_setup(context: LaunchContext):
    load_gripper = context.perform_substitution(LaunchConfiguration('load_gripper'))
    world = context.perform_substitution(LaunchConfiguration('world'))

    robot_description, robot_description_semantic = build_descriptions(context, load_gripper)
    kinematics_yaml = load_yaml('franka_fr3_moveit_config', 'config/kinematics.yaml')

    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '
                                'default_planner_request_adapters/ResolveConstraintFrames '
                                'default_planner_request_adapters/FixWorkspaceBounds '
                                'default_planner_request_adapters/FixStartStateBounds '
                                'default_planner_request_adapters/FixStartStateCollision '
                                'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_yaml = load_yaml('franka_fr3_moveit_config', 'config/ompl_planning.yaml')
    if ompl_planning_yaml:
        ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    # Trajectory execution / controller manager. Required to avoid a crash in
    # move_group's execution manager even when we only plan (not execute).
    moveit_controllers_yaml = load_yaml(
        'franka_fr3_moveit_config', 'config/fr3_controllers.yaml')
    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_controllers_yaml,
        'moveit_controller_manager':
            'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }
    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }
    # True (default) when paired with the Gazebo sim; set false to run MoveIt
    # standalone (no /clock).
    use_sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}

    move_group = Node(
        package='moveit_ros_move_group', executable='move_group', output='screen',
        parameters=[robot_description, robot_description_semantic, kinematics_yaml,
                    ompl_planning_pipeline_config, trajectory_execution,
                    moveit_controllers, planning_scene_monitor_parameters,
                    use_sim_time])

    rviz_config = os.path.join(
        get_package_share_directory('franka_fr3_moveit_config'), 'rviz', 'moveit.rviz')
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='log',
        arguments=['-d', rviz_config],
        parameters=[robot_description, robot_description_semantic, kinematics_yaml,
                    ompl_planning_pipeline_config, use_sim_time],
        condition=IfCondition(LaunchConfiguration('rviz')))

    # NOTE: no use_sim_time here on purpose -- the publisher emits static
    # geometry on a wall-clock timer. With sim time and no /clock yet, the timer
    # would never fire and the table/box would never be published.
    scene = Node(
        package='franka_warehouse_world', executable='publish_planning_scene.py',
        name='warehouse_planning_scene', output='screen',
        parameters=[{
            'world': world,
            'frame_id': 'fr3_link0',
            'config': os.path.join(
                get_package_share_directory('franka_warehouse_world'),
                'config', 'scene.yaml'),
        }])

    return [move_group, rviz, scene]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='small',
            description="Which box size to add to the scene: 'small' or 'large'."),
        DeclareLaunchArgument(
            'load_gripper', default_value='false',
            description='Must match the value passed to warehouse.launch.py.'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Start MoveIt RViz (MotionPlanning display).'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the Gazebo /clock. Set false to run MoveIt without '
                        'the sim.'),
        OpaqueFunction(function=launch_setup),
    ])
