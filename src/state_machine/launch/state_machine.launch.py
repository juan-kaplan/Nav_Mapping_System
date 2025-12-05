import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("state_machine")

    default_rviz_config_path = os.path.join(pkg_share, "rviz", "state_machine.rviz")

    # Declare an argument to allow changing the RViz config from the command line
    rviz_arg = DeclareLaunchArgument(
        name="rviz_config",
        default_value=default_rviz_config_path,
        description="Absolute path to rviz config file",
    )

    # Your custom Map Publisher Node
    map_publisher_node = Node(
        package="state_machine",
        executable="map_publisher",
        name="map_publisher",
        output="screen",
    )

    likelihood_map_pub_node = Node(
        package="state_machine",
        executable="likelihood_map_publisher",
        name="likelihood_map_publisher",
        output="screen",
    )

    localization_node = Node(
        package="state_machine",
        executable="localization",
        name="localization",
        output="screen",
    )

    # RViz2 Node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
    )

    return LaunchDescription(
        [
            rviz_arg,
            map_publisher_node,
            rviz_node,
            likelihood_map_pub_node,
            localization_node,
        ]
    )
