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
from launch.conditions import IfCondition
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
    publish_map_tf = LaunchConfiguration('publish_map_tf', default='false')
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
                ('/tf', f'/{namespace}/tf' + ('_rsp' if publish_map_tf.perform(context).lower() == 'true' else '')), # isolate dynamic tf topic
                ('/tf_static', f'/{namespace}/tf_static'),
            ]
        ),

        Node(
            package='tb3_multi_launch',
            executable='pose_node',
            name='pose_node',
            namespace=namespace,
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'map_frame': 'odom', # TODO: we might want this to be an option. we publish odom -> base_footprint instead for the sake of being proper (since normally it's map->odom and odom->base_footprint)
                'robot_frame': 'base_footprint',
                'robot_name': namespace
            }],
            remappings=[
                ('/tf', f'/{namespace}/tf'),
                ('/tf_static', f'/{namespace}/tf_static'), # probably not needed but let's throw this in too
            ],
            condition=IfCondition(publish_map_tf)
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map2odom',
            namespace=namespace,
            output='screen',
            arguments = [
                '--x', '0', '--y', '-0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'map',
                '--child-frame-id', 'odom'
            ],
            remappings=[
                ('/tf', f'/{namespace}/tf'),
                ('/tf_static', f'/{namespace}/tf_static'),
            ],
            condition=IfCondition(publish_map_tf)
        ),

        # these are not needed, and exist purely for aesthetics
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='caster2wheel_left',
            namespace=namespace,
            output='screen',
            arguments = [
                '--x', '0.177', '--y', '-0.027', '--z', '0.080',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'caster_back_left_link',
                '--child-frame-id', 'wheel_left_link'
            ],
            remappings=[
                ('/tf', f'/{namespace}/tf'),
                ('/tf_static', f'/{namespace}/tf_static'),
            ],
            condition=IfCondition(publish_map_tf)
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='caster2wheel_right',
            namespace=namespace,
            output='screen',
            arguments = [
                '--x', '0.177', '--y', '-0.027', '--z', '-0.080',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'caster_back_right_link',
                '--child-frame-id', 'wheel_right_link'
            ],
            remappings=[
                ('/tf', f'/{namespace}/tf'),
                ('/tf_static', f'/{namespace}/tf_static'),
            ],
            condition=IfCondition(publish_map_tf)
        ),
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
            'publish_map_tf',
            default_value='false',
            description='Directly publish map -> base_footprint transform from Gazebo'
        ),

        DeclareLaunchArgument(
            'namespace',
            description='The namespace of the robot to be spawned'
        ),

        OpaqueFunction(function=launch_node)
    ])
