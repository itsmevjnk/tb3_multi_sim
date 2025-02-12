# Copyright 2019 Open Source Robotics Foundation, Inc.
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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

MODEL_DIRS = {
    'waffle': os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'models', 'turtlebot3_waffle'),
    'waffle_pi': os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'models', 'turtlebot3_waffle_pi'),
    'burger': os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'models', 'turtlebot3_burger'),
    'waffle_bm': os.path.join(get_package_share_directory('tb3_multi_launch'), 'models', 'turtlebot3_waffle'),
}

def launch_spawner(context):
    TURTLEBOT3_MODEL = LaunchConfiguration('model', default='waffle').perform(context)
    urdf_path = os.path.join(
        MODEL_DIRS[TURTLEBOT3_MODEL],
        'model.sdf'
    )

    x_pose = LaunchConfiguration('x_pose', default='0.0').perform(context)
    y_pose = LaunchConfiguration('y_pose', default='0.0').perform(context)
    yaw_pose = LaunchConfiguration('yaw_pose', default='0.0').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)
    
    return [
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name=f'{namespace}_spawner',
            arguments=[
                '-entity', namespace,
                '-file', urdf_path,
                '-x', x_pose,
                '-y', y_pose,
                '-z', '0.01',
                '-Y', yaw_pose,
                '-robot_namespace', namespace
            ],
            output='screen'
        )
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'x_pose', default_value='0.0',
            description='Initial X position (in metres)'
        ),

        DeclareLaunchArgument(
            'y_pose', default_value='0.0',
            description='Initial Y position (in metres)'
        ),

        DeclareLaunchArgument(
            'yaw_pose', default_value='0.0',
            description='Initial yaw angle (in radians)'
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

        OpaqueFunction(function=launch_spawner)
    ])
