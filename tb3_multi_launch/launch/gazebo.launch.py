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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

WORLDS = {
    'empty': os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'worlds', 'empty_world.world'),
    'house': os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'worlds', 'turtlebot3_house.world'),
    'world': os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'worlds', 'turtlebot3_world.world'),
    'dqn1': os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'worlds', 'turtlebot3_dqn_stage1.world'),
    'dqn2': os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'worlds', 'turtlebot3_dqn_stage2.world'),
    'dqn3': os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'worlds', 'turtlebot3_dqn_stage3.world'),
    'dqn4': os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'worlds', 'turtlebot3_dqn_stage4.world'),
    'world_2x': os.path.join(get_package_share_directory('tb3_multi_launch'), 'worlds', 'turtlebot3_world_2x.world'),
    'world_4x': os.path.join(get_package_share_directory('tb3_multi_launch'), 'worlds', 'turtlebot3_world_4x.world'),
}

def launch_gzserver(context, pkg_gazebo_ros):
    world = WORLDS[LaunchConfiguration('world', default='empty').perform(context)]

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world}.items()
        )
    ]

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    return LaunchDescription([
        OpaqueFunction(function=launch_gzserver, args=[pkg_gazebo_ros]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            )
        )
    ])
