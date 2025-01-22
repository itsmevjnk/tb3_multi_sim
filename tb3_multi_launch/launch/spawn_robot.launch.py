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
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def spawn_robot(context):
    launch_file_dir = os.path.join(get_package_share_directory('tb3_multi_launch'), 'launch')
    bridge_launch_dir = os.path.join(get_package_share_directory('tb3_domain_bridge'), 'launch')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true').perform(context)
    x_pose = LaunchConfiguration('x_pose', default='-2.0').perform(context)
    y_pose = LaunchConfiguration('y_pose', default='-0.5').perform(context)

    model = LaunchConfiguration('model', default='waffle').perform(context)
    namespace = LaunchConfiguration('namespace', default='').perform(context) # blank = random
    domain = LaunchConfiguration('domain').perform(context)

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
                'model': model
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir, 'tb3_spawner.launch.py')
            ),
            launch_arguments={
                'x_pose': x_pose,
                'y_pose': y_pose,
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
                'use_sim_time': use_sim_time
            }.items()
        )
    ]

def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('tb3_multi_launch'), 'launch')
    bridge_launch_dir = os.path.join(get_package_share_directory('tb3_domain_bridge'), 'launch')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='-0.5')

    model = LaunchConfiguration('model', default='waffle')
    namespace = LaunchConfiguration('namespace', default='') # blank = random
    domain = LaunchConfiguration('domain')

    return LaunchDescription([
        OpaqueFunction(function=spawn_robot)
    ])
