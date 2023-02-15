# Copyright 2021 Intelligent Robotics Lab
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
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import yaml


def generate_launch_description():
    nocompila_dir = get_package_share_directory('nc_bt_patrolling')

    config = os.path.join(nocompila_dir, 'config', 'params.yaml')

    with open(config, 'r') as stream:
        try:
            conf = (yaml.safe_load(stream))

        except yaml.YAMLError as exc:
            print(exc)

    patrolling_cmd = Node(
        package='nc_bt_patrolling',
        executable='patrolling_main',
        parameters=[{
          'use_sim_time': conf['use_sim_time'],
          'robot': conf['nc_bt_patrolling']['robot'],
          'type': conf['nc_bt_patrolling']['type'],
          'wp_1': conf['nc_bt_patrolling']['way_points']['wp_1'],
          'wp_2': conf['nc_bt_patrolling']['way_points']['wp_2'],
          'wp_3': conf['nc_bt_patrolling']['way_points']['wp_3'],
          'wp_4': conf['nc_bt_patrolling']['way_points']['wp_4']
        }],
        remappings=[
          ('input_scan', conf['nc_bt_patrolling']['input_scan']),
          ('output_vel', conf['nc_bt_patrolling']['output_vel'])
        ],
        output='screen'
    )

    ld = LaunchDescription()

    # Add any actions
    # ld.add_action(tracking_cmd)
    ld.add_action(patrolling_cmd)

    return ld
