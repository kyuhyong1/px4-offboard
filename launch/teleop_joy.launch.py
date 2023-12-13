#!/usr/bin/env python
__author__ = "Kyuhyong You"
__contact__ = "kyuhyong.you@nearthlab.com"

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_dir = get_package_share_directory('uds_mmu_teleop')
    # bash_script_path = os.path.join(package_dir, 'scripts', 'TerminatorScript.sh')
    return LaunchDescription([
        # ExecuteProcess(cmd=['bash', bash_script_path], output='screen'),
        #Node(
        #    package='uds_mmu_teleop',
        #    namespace='uds_mmu_teleop',
        #    executable='visualizer',
        #    name='visualizer'
        #),
        Node(
            package='uds_mmu_teleop',
            namespace='uds_mmu_teleop',
            executable='teleop_joy',
            name='teleop_joy',
            prefix='gnome-terminal --',
        ),
        Node(
            package='uds_mmu_teleop',
            namespace='uds_mmu_teleop',
            executable='teleop_commander',
            name='teleop_commander'
        ),
        #Node(
        #    package='rviz2',
        #    namespace='',
        #    executable='rviz2',
        #    name='rviz2',
        #    arguments=['-d', [os.path.join(package_dir, 'visualize.rviz')]]
        #),
        Node(
            package='joy',
            namespace='',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'device': '/dev/input/js0',
                'deadzone': 0.1,
                'autorepeat_rate': 20.0,
            }]
        )
    ])
