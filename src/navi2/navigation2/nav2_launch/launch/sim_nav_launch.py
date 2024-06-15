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
    # ROBOT_MODEL = os.environ['ROBOT_MODEL']
    # Get the launch directory
    pkg_dir = get_package_share_directory('nav2_launch')
    launch_dir = os.path.join(pkg_dir, 'launch')
    nav_launch_dir = os.path.join(launch_dir, 'navi', 'simu')
    params_dir = os.path.join(pkg_dir, 'params', 'simu')

    # Create the launch configuration variables
    log_level = LaunchConfiguration('log_level')
    use_sim_time = LaunchConfiguration('use_sim_time')
    navi_params_file = LaunchConfiguration('navi_params_file')
    colli_params_file = LaunchConfiguration('colli_params_file')

    # Declare the launch arguments
    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log info')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_navi_params_file_cmd = DeclareLaunchArgument(
        'navi_params_file',
        default_value=os.path.join(params_dir, 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_colli_params_file_cmd = DeclareLaunchArgument(
        'colli_params_file',
        default_value=os.path.join(params_dir, 'collision_monitor_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')
        
    # Specify the actions
    navi_cmd_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'navi', 'collision_monitor_launch.py')),
            launch_arguments={'log_level': log_level,
                              'use_sim_time': use_sim_time,
                              'params_file': colli_params_file}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_launch_dir, 'navigation_launch.py')),
            launch_arguments={'log_level': log_level,
                              'use_sim_time': use_sim_time,  
                              'params_file': navi_params_file}.items()),
  
    ])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_use_sim_time_cmd)    
    ld.add_action(declare_navi_params_file_cmd)
    ld.add_action(declare_colli_params_file_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(navi_cmd_group)

    return ld
