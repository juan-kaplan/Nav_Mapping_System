#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('ubicate_nene')

    # RViz configuration (slam.rviz in your rviz/ folder)
    rviz_config_file = LaunchConfiguration(
        'rviz_config',
        default=os.path.join(pkg_share, 'rviz', 'slam.rviz')
    )

    num_particles_arg = DeclareLaunchArgument(
        "num_particles",
        default_value="1000",
        description="Number of particles for the filter"
    )

    # --- FastSLAM node ---
    fast_slam_node = Node(
        package='ubicate_nene',        # change if your package name is different
        executable='fastslam',    # must match the installed executable name
        name='fast_slam_node',
        output='screen',
        parameters=[{"num_particles": LaunchConfiguration("num_particles")}]
    )

    # --- RViz2 with slam.rviz ---
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    ld = LaunchDescription()
    ld.add_action(fast_slam_node)
    ld.add_action(rviz2_node)

    return ld
