#!/usr/bin/env python3
# Copyright (C) 2022 Toyota Motor Corporation
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution)
from tmc_launch_ros_utils.tmc_launch_ros_utils import load_robot_description


def declare_arguments(context, robot_name):
    declared_arguments = []
    robot_name_str = context.perform_substitution(robot_name)
    if robot_name_str == 'hsrb':    
        declared_arguments.append(DeclareLaunchArgument('description_package', default_value='hsrb_description',
                                                        description='Description package with robot URDF/oo files.'))
        declared_arguments.append(DeclareLaunchArgument('description_file', default_value='hsrb4s.urdf.xacro',
                                                        description='URDF/XACRO description file with the robot.'))
        declared_arguments.append(DeclareLaunchArgument('collision_file', default_value='collision_pair_hsrb.xml',
                                                        description='collision_pair_hsrb file with the robot.'))
    else:
        declared_arguments.append(DeclareLaunchArgument('description_package', default_value='hsrc_description',
                                                        description='Description package with robot URDF/oo files.'))
        declared_arguments.append(DeclareLaunchArgument('description_file', default_value='hsrc1s.urdf.xacro',
                                                        description='URDF/XACRO description file with the robot.'))
        declared_arguments.append(DeclareLaunchArgument('collision_file', default_value='collision_pair_hsrc.xml',
                                                        description='collision_pair_hsrc file with the robot.'))
    return declared_arguments


def launch_setup(context):

    robot_description = load_robot_description(xacro_arg='gazebo_sim:=True')

    ihsrb_node = Node(package='hsrb_interface_py',
                      executable='ihsrb.py',
                      name='ihsrb_node',
                      parameters=[robot_description],
                      output='screen')

    nodes = [ihsrb_node]

    return nodes


def generate_launch_description():

    return LaunchDescription(
        [DeclareLaunchArgument('robot_name', default_value='hsrb', description='Robot Name.'),
         OpaqueFunction(function=declare_arguments,
                        args=[LaunchConfiguration('robot_name')]),       
         OpaqueFunction(function=launch_setup)])

