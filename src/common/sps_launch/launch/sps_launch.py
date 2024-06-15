# Copyright (c) 2018 Intel Corporation
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
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    ROBOT_MODEL = os.environ['ROBOT_MODEL']
    # Get the launch directory
    pkg_dir = get_package_share_directory('sps_launch')
    params_dir = os.path.join(pkg_dir, 'params', ROBOT_MODEL)

    
    map_server_pkg_dir = get_package_share_directory('map_server')
    map_server_launch_dir = os.path.join(map_server_pkg_dir, 'launch')
    
    server_interface_pkg_dir = get_package_share_directory('server_interface')
    server_interface_launch_dir = os.path.join(server_interface_pkg_dir, 'launch')
    
    monitor_manager_dir = get_package_share_directory('monitor_manager')
    monitor_manager_launch_dir = os.path.join(monitor_manager_dir, 'launch')

    semantic_area_servere_pkg_dir = get_package_share_directory('semantic_area_server')
    semantic_area_server_launch_dir = os.path.join(semantic_area_servere_pkg_dir, 'launch')

    velocity_manager_pkg_dir = get_package_share_directory('velocity_manager')
    velocity_manager_launch_dir = os.path.join(velocity_manager_pkg_dir, 'launch')
    
    params_manager_pkg_dir = get_package_share_directory('params_manager')


    # Create the launch configuration variables
    log_level = LaunchConfiguration('log_level')
    sps_params_file = LaunchConfiguration('sps_params_file')


    # Declare the launch arguments
    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log info')


    declare_sps_params_file_cmd = DeclareLaunchArgument(
        'sps_params_file',
        default_value=os.path.join(params_dir, 'sps_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')


        
    # Specify the actions
    sps_cmd_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(map_server_launch_dir, 'map_server.launch.py')),
            launch_arguments={'log_level': log_level,
                              'params_file': sps_params_file}.items()),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(server_interface_launch_dir, 'server_interface.launch.py')),
            launch_arguments={'log_level': log_level,
                              'params_file': sps_params_file}.items()),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(monitor_manager_launch_dir, 'monitor_manager.launch.py')),
            launch_arguments={'log_level': log_level,
                              'params_file': sps_params_file}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(semantic_area_server_launch_dir, 'semantic_area_server.launch.py')),
            launch_arguments={'log_level': log_level,
                              'params_file': sps_params_file}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(velocity_manager_launch_dir, 'velocity_manager.launch.py')),
            launch_arguments={'log_level': log_level,
                              'params_file': sps_params_file}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(params_manager_pkg_dir, 'params_manager.launch.py')),
            launch_arguments={'log_level': log_level,
                              'params_file': sps_params_file}.items()),
    ])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_log_level_cmd)  
    ld.add_action(declare_sps_params_file_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(sps_cmd_group)

    return ld
