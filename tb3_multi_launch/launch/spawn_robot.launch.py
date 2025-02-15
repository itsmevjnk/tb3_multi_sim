#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Joep Tool

import os
import random

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, DeclareLaunchArgument, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def spawn_robot(context):
    launch_file_dir = os.path.join(get_package_share_directory('tb3_multi_launch'), 'launch')
    bridge_launch_dir = os.path.join(get_package_share_directory('tb3_domain_bridge'), 'launch')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true').perform(context)
    x_pose = LaunchConfiguration('x_pose', default='-2.0').perform(context)
    y_pose = LaunchConfiguration('y_pose', default='-0.5').perform(context)
    yaw_pose = LaunchConfiguration('yaw_pose', default='0.0').perform(context)

    model = LaunchConfiguration('model', default='waffle').perform(context)
    namespace = LaunchConfiguration('namespace', default='').perform(context) # blank = random
    domain = LaunchConfiguration('domain').perform(context)
    
    publish_map_tf = LaunchConfiguration('publish_map_tf', default='false')

    if namespace == '':
        namespace = model + '_' + ''.join([random.choice('0123456789abcdef') for x in range(8)])

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'namespace': namespace,
                'model': model,
                'publish_map_tf': publish_map_tf
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir, 'tb3_spawner.launch.py')
            ),
            launch_arguments={
                'x_pose': x_pose,
                'y_pose': y_pose,
                'yaw_pose': yaw_pose,
                'namespace': namespace,
                'model': model
            }.items()
        ),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(bridge_launch_dir, 'bridge_launch.xml')
            ),
            launch_arguments={
                'namespace': namespace,
                'domain': domain,
                'use_sim_time': use_sim_time,
                'publish_odom_tf': str(publish_map_tf.perform(context).lower() == 'false')
            }.items()
        ),

        Node(
            package='tb3_multi_launch',
            executable='delete_watch_node',
            name='delete_watch_node',
            output='screen',
            parameters=[{
                'name': namespace
            }],
            remappings=[],
            on_exit=Shutdown()
        )
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                                description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('x_pose', default_value='-2.0',
                                description='Initial x position of the robot'),
        DeclareLaunchArgument('y_pose', default_value='-0.5',
                                description='Initial y position of the robot'),
        DeclareLaunchArgument('yaw_pose', default_value='0.0',
                                description='Initial yaw angle of the robot'),
        DeclareLaunchArgument('model', default_value='waffle',
                                description='Robot model type (waffle, burger or waffle_pi)'),
        DeclareLaunchArgument('namespace', default_value='',
                                description='The robot\'s name (and its namespace), random by default'),
        DeclareLaunchArgument('domain', description='The ROS2 domain to bridge the robot\'s topics to'),

        DeclareLaunchArgument(
            'publish_map_tf',
            default_value='false',
            description='Directly publish map -> base_footprint transform from Gazebo'
        ),

        OpaqueFunction(function=spawn_robot)
    ])
