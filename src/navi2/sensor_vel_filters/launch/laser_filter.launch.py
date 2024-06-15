import os
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution,LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="sensor_vel_filters",
            executable="vel_laser_filter",
            name='depthscan_filter',
            remappings=[('scan', '/depth/scan'),
                        ('/cmd_vel','/navigation/cmd_vel'),
                        ('/scan_filter','/depthscan_filter')],
            parameters=[
                {"truncate_range": 1.1},
                {"simutime": 2.0},
                {"upper_threshold_max": 1.5},
                {"upper_threshold_min": 1.0},
                {"lower_threshold": 0.0},
            ]
        ),
        Node(
            package="sensor_vel_filters",
            executable="vel_laser_filter",
            name='laserscan_filter',
            remappings=[#('scan', '/sonar_scan'),
                        ('/cmd_vel','/navigation/cmd_vel'),
                        ('/scan_filter','/laserscan_filter')],
            parameters=[
                {"truncate_range": 1.0},
                {"simutime": 2.0},
                {"upper_threshold_max": 3.0},
                {"upper_threshold_min": 1.0},
                {"lower_threshold": 0.0},
            ]
        )
    ])
