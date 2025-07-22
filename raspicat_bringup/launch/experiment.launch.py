# Copyright 2023 RT Corporation
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

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    config = LaunchConfiguration(
        'config')
    joy_dev = LaunchConfiguration('joy_dev')
    output_vel = LaunchConfiguration('output_vel')

    joy = GroupAction(
        actions=[
            DeclareLaunchArgument('config', default_value=[
                TextSubstitution(text=os.path.join(
                    get_package_share_directory('raspicat_bringup'), 'config', '')),
                'experiment', TextSubstitution(text='.param.yaml')]),
            DeclareLaunchArgument(
                'joy_dev', default_value='/dev/input/js0'),
            DeclareLaunchArgument(
                'output_vel', default_value='cmd_vel'),

            Node(
                package='joy_linux',
                executable='joy_linux_node',
                name='joy_node',
                parameters=[{
                    'dev': joy_dev,
                    'deadzone': 0.05,
                    'autorepeat_rate': 20.0,
                }]),
            Node(
                package='teleop_twist_joy',
                executable='teleop_node',
                name='teleop_twist_joy_node',
                parameters=[config],
                remappings={
                    ('/cmd_vel', 'joy_vel')},
            ),
            Node(
                package="relief_stopper",
                executable="relief_stop",
                name="relief_stopper",
                parameters=[config],
                output="screen",
            ),
            Node(
                package="relief_detector",
                executable="relief_detection",
                name="relief_detector",
                parameters=[config],
                output="screen",
            ),
            Node(
                package='raspicat',
                executable='velocity_smoother_controller',
                name='velocity_smoother_controller_node',
                parameters=[config],
                remappings={
                    ('/input_vel',  'relief_vel'),
                    ('/output_vel', output_vel)},
            ),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother_node',
                parameters=[config],
                remappings={
                    ('/cmd_vel', 'control_vel')},
            ),
            ]
        )

    ld = LaunchDescription()

    ld.add_action(joy)

    return ld
