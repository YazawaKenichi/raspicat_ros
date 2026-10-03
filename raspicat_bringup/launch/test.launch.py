import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['serial', '-b', '115200', '--dev', '/dev/ttyACM0', '-v6'],
        output='screen',
    )

    experiment = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('raspicat_bringup'), 'launch'),
            '/experiment.launch.py'])
    )

    ld = LaunchDescription()

    ld.add_action(micro_ros_agent)
    ld.add_action(experiment)

    return ld
