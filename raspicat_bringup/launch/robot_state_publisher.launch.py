# Copyright 2022-2023 RT Corporation
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
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    lidar_frame = LaunchConfiguration('lidar_frame')
    use_joint_state_publisher = LaunchConfiguration(
        'use_joint_state_publisher')

    declare_lidar_frame = DeclareLaunchArgument(
        'lidar_frame',
        default_value='lidar_link',
        description='Set lidar link name.')
    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Set namespace for tf tree.')
    declare_use_joint_state_publisher = DeclareLaunchArgument(
        'use_joint_state_publisher',
        default_value='True',
        description='Set joint_state_publisher for tf tree.')

    xacro_file = os.path.join(get_package_share_directory(
        'raspicat_description'), 'urdf', 'raspicat.urdf.xacro')

    params = {'robot_description':
              Command(['xacro ', xacro_file,
                       ' lidar_frame:=', lidar_frame, ]),
              'frame_prefix': [namespace, '/']}

    push_ns = PushRosNamespace([namespace])

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        condition=IfCondition(use_joint_state_publisher))

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[params])

    ld = LaunchDescription()

    ld.add_action(declare_namespace)
    ld.add_action(declare_lidar_frame)
    ld.add_action(declare_use_joint_state_publisher)
    ld.add_action(push_ns)
    ld.add_action(joint_state_publisher)
    ld.add_action(robot_state_publisher)

    return ld
