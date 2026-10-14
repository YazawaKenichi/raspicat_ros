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
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    config = LaunchConfiguration(
        'config')
    joy_dev = LaunchConfiguration('joy_dev')
    output_vel = LaunchConfiguration('output_vel')
    micro_ros_dev = LaunchConfiguration('micro_ros_dev')
    view = LaunchConfiguration('view')

    experiment = GroupAction(
        actions=[
            DeclareLaunchArgument('config', default_value=[
                TextSubstitution(text=os.path.join(
                    get_package_share_directory('raspicat_bringup'), 'config', '')),
                'experiment', TextSubstitution(text='.param.yaml')]),
            DeclareLaunchArgument(
                'joy_dev', default_value='/dev/input/js0'),
            DeclareLaunchArgument(
                'output_vel', default_value='cmd_vel'),
            DeclareLaunchArgument(
                'micro_ros_dev',
                default_value='/dev/serial/by-id/usb-Raspberry_Pi_Pico_E6609CB2D327AF2A-if00'),
            DeclareLaunchArgument(
                'view', default_value='true'),

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
                package="relief_detector",
                executable="relief_detection",
                name="relief_detector",
                parameters=[config],
                output="screen",
            ),
            # joy_vel -> relief_vel: 停止中は smoother の入力もゼロにして再発進を滑らかにする
            Node(
                package="relief_stopper",
                executable="relief_stop",
                name="relief_stopper_pre",
                parameters=[config],
                output="screen",
            ),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother_node',
                parameters=[config],
                remappings={
                    ('/cmd_vel', 'relief_vel'),
                    ('/cmd_vel_smoothed', 'smoothed_vel')},
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_velocity_smoother',
                parameters=[config],
            ),
            # smoothed_vel -> output_vel: 加減速制限を介さず即時に停止させる
            Node(
                package="relief_stopper",
                executable="relief_stop",
                name="relief_stopper_post",
                parameters=[config, {'processed_cmd_vel_topic_name': output_vel}],
                output="screen",
            ),
            Node(
                package='micro_ros_agent',
                executable='micro_ros_agent',
                name='micro_ros_agent',
                arguments=[
                    'serial',
                    '-b', '115200',
                    '--dev', micro_ros_dev,
                    '-v6',
                ],
                output='screen',
            ),
            Node(
                package='phone_camera',
                executable='phone_camera_node',
                name='phone_camera',
                output='screen',
                parameters=[config],
            ),
            Node(
                package='phone_camera',
                executable='image_viewer_node',
                name='image_viewer',
                output='screen',
                condition=IfCondition(view),
            ),
            ]
        )

    ld = LaunchDescription()

    ld.add_action(experiment)

    return ld
