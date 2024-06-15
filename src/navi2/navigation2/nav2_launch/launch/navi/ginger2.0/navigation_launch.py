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
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_launch')
    log_level = LaunchConfiguration('log_level')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_respawn = LaunchConfiguration('use_respawn')

    lifecycle_nodes = ['controller_server',
                       'planner_server',
                       'local_planner_server',
                       'recoveries_server',
                       'bt_navigator',
                       'task_manager',
                       'sensor_filter']

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static'),
                  ('/cmd_vel', 'cmd_vel_raw'),
                  ('/goal_pose', '/move_base_simple/goal'),
                  ('/map', '/occ_map')]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart}

    path_hari_params = "/home/ginger/sps_nav2_params/nav2_params.yaml"
    if os.path.exists(path_hari_params):
        print(path_hari_params," is exists")
        params_path = path_hari_params
    else:
        print(path_hari_params," is not exists")
        params_path = os.path.join(bringup_dir, 'params', 'patrol', 'nav2_params.yaml')


    configured_params = RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True)

    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        
        DeclareLaunchArgument(
            'log_level',
            default_value="INFO",
            description='Log level config'),

        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Top-level namespace'),

        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack'),

        DeclareLaunchArgument(
            'params_file',
            default_value=params_path,
            description='Full path to the ROS2 parameters file to use'),

        DeclareLaunchArgument(
            'use_respawn', default_value='True',
            description='Whether to respawn if a node crashes. Applied when composition is disabled.'),

        Node(
            package='nav2_controller',
            executable='controller_server',
            output='full',
            respawn=use_respawn,
            parameters=[configured_params],
            remappings=remappings,
            arguments=['--ros-args', '--log-level', log_level]),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='full',
            respawn=use_respawn,
            parameters=[configured_params],
            remappings=remappings,
            arguments=['--ros-args', '--log-level', log_level]),

        Node(
            package='nav2_local_planner',
            executable='local_planner_server',
            name='local_planner_server',
            output='full',
            respawn=use_respawn,
            parameters=[configured_params],
            remappings=remappings,
            arguments=['--ros-args', '--log-level', log_level]),

        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='full',
            respawn=use_respawn,
            parameters=[configured_params],
            remappings=remappings,
            arguments=['--ros-args', '--log-level', log_level]),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='full',
            respawn=use_respawn,
            parameters=[configured_params],
            remappings=remappings,
            arguments=['--ros-args', '--log-level', log_level]),

        Node(
            package='nav2_task_manager',
            executable='task_manager',
            name='task_manager',
            output='full',
            respawn=use_respawn,
            parameters=[configured_params],
            remappings=remappings,
            arguments=['--ros-args', '--log-level', log_level]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='full',
            respawn=use_respawn,
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}],
            arguments=['--ros-args', '--log-level', log_level]),

        Node(
            package="sensor_vel_filters",
            executable="sensor_filter",
            name='sensor_filter',
            output='full',
            respawn=use_respawn,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level]),



    ])
