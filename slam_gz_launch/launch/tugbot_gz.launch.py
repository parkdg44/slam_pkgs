# Copyright 2022 Open Source Robotics Foundation, Inc.
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
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    this_package = get_package_share_directory('slam_gz_launch')
    project_name = 'tugbot'
    gz_root_path = os.path.join(this_package, 'world', project_name)
    sdf_file = os.path.join(gz_root_path, 'world.sdf')
    rviz_file = os.path.join(gz_root_path, 'rviz.rviz')
    bridge_config_file = os.path.join(gz_root_path, 'bridge.yaml')

    gz_process = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', sdf_file],
        additional_env={'GZ_SIM_RESOURCE_PATH': f'{this_package}'}
    )

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_config_file
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_file],
        parameters=[{"use_sim_time": True}],
    )

    ld.add_action(gz_process)
    ld.add_action(bridge_node)
    ld.add_action(rviz_node)

    return ld
