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
# Authors: Darby Lim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

URDF_FILES = {
    'waffle': os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'urdf', 'turtlebot3_waffle.urdf'),
    'waffle_pi': os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'urdf', 'turtlebot3_waffle_pi.urdf'),
    'burger': os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'urdf', 'turtlebot3_burger.urdf'),
    'waffle_bm': os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'urdf', 'turtlebot3_waffle.urdf'),
}

def launch_node(context):
    TURTLEBOT3_MODEL = LaunchConfiguration('model', default='waffle').perform(context)

    urdf_path = URDF_FILES[TURTLEBOT3_MODEL]
    urdf_file_name = os.path.basename(urdf_path)
    print('urdf_file_name : {}'.format(urdf_file_name))
    
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    namespace = LaunchConfiguration('namespace').perform(context)

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=namespace,
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_desc
            }],
            remappings=[
                ('/tf', f'/{namespace}/tf'),
                ('/tf_static', f'/{namespace}/tf_static'),
            ]
        )
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        DeclareLaunchArgument(
            'model',
            default_value='waffle',
            description='The Turtlebot 3 model to be used (waffle, burger or waffle_pi)'
        ),

        DeclareLaunchArgument(
            'namespace',
            description='The namespace of the robot to be spawned'
        ),

        OpaqueFunction(function=launch_node)
    ])
