# Copyright 2019 Open Source Robotics Foundation, Inc.
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

"""Launch a talker and a listener in a component container."""

import launch
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Environment
    ROBOT_MODEL = os.environ['ROBOT_MODEL']
    package_dir = get_package_share_directory('nav2_launch')
    params_dir = os.path.join(package_dir, 'params', ROBOT_MODEL)
    collision_params_file_path = os.path.join(params_dir, 'collision_monitor_params.yaml')
    harix_navigation_params_file_path = "/home/ginger/sps_nav2_params/nav2_params.yaml"
    if os.path.exists(harix_navigation_params_file_path):
        print("Harix params file \'%s\' is exists!" % harix_navigation_params_file_path)
        navigation_params_file_path = harix_navigation_params_file_path
    else:
        print("Harix params file \'%s\' is not exists!" % harix_navigation_params_file_path)
        navigation_params_file_path = os.path.join(params_dir, 'nav2_params.yaml')

    print("Loading collision  params from \'%s\' for robot %s" % (collision_params_file_path, ROBOT_MODEL))
    print("Loading navigation params from \'%s\' for robot %s" % (navigation_params_file_path, ROBOT_MODEL))

    # Constant parameters
    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
        ('/cmd_vel', 'cmd_vel_raw'),
        ('/goal_pose', '/move_base_simple/goal'),
        ('/map', '/occ_map')
    ]

    # Launch arguments
    # 1. Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    navigation_params_file = LaunchConfiguration('navigation_params_file')
    collision_params_file = LaunchConfiguration('collision_params_file')

    # 2. Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack')

    declare_navi_params_file_cmd = DeclareLaunchArgument(
        'navigation_params_file',
        default_value=navigation_params_file_path,
        description='Full path to the ROS2 parameters file to use for navigation nodes')

    declare_collision_params_file_cmd = DeclareLaunchArgument(
        'collision_params_file',
        default_value=collision_params_file_path,
        description='Full path to the ROS2 parameters file to use for collision nodes')

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart
    }

    collision_monitor_configured_params = RewrittenYaml(
        source_file=collision_params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    navigation_configured_params = RewrittenYaml(
        source_file=navigation_params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    lifecycle_managed_nodes_map = {}
    if ROBOT_MODEL == "ginger":
        lifecycle_managed_nodes_map = {
            'collision_monitor': ComposableNode(
                package='nav2_collision_monitor',
                plugin='nav2_collision_monitor::CollisionMonitor',
                name='collision_monitor',
                parameters=[
                    collision_monitor_configured_params
                ],
            ),
            'controller_server': ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                remappings=remappings,
                parameters=[
                    navigation_configured_params
                ],
            ),
            'planner_server': ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                remappings=remappings,
                parameters=[
                    navigation_configured_params
                ],
            ),
            'recoveries_server': ComposableNode(
                package='nav2_recoveries',
                plugin='recovery_server::RecoveryServer',
                name='recoveries_server',
                remappings=remappings,
                parameters=[
                    navigation_configured_params
                ],
            ),
            'bt_navigator': ComposableNode(
                package='nav2_bt_navigator',
                plugin='nav2_bt_navigator::BtNavigator',
                name='bt_navigator',
                remappings=remappings,
                parameters=[
                    navigation_configured_params
                ],
            ),
            'task_manager': ComposableNode(
                package='nav2_task_manager',
                plugin='nav2_task_manager::TaskManager',
                name='task_manager',
                remappings=remappings,
                parameters=[
                    navigation_configured_params
                ],
            ),
            'sensor_filter': ComposableNode(
                package='sensor_vel_filters',
                plugin='sensor_vel_filters::SensorFilter',
                name='sensor_filter',
                remappings=remappings,
                parameters=[
                    navigation_configured_params
                ],
            ),
        }
    elif ROBOT_MODEL == "ginger2.0":
        lifecycle_managed_nodes_map = {
            'collision_monitor': ComposableNode(
                package='nav2_collision_monitor',
                plugin='nav2_collision_monitor::CollisionMonitor',
                name='collision_monitor',
                parameters=[
                    collision_monitor_configured_params
                ],
            ),
            'controller_server': ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                remappings=remappings,
                parameters=[
                    navigation_configured_params
                ],
            ),
            'planner_server': ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                remappings=remappings,
                parameters=[
                    navigation_configured_params
                ],
            ),
            'recoveries_server': ComposableNode(
                package='nav2_recoveries',
                plugin='recovery_server::RecoveryServer',
                name='recoveries_server',
                remappings=remappings,
                parameters=[
                    navigation_configured_params
                ],
            ),
            'bt_navigator': ComposableNode(
                package='nav2_bt_navigator',
                plugin='nav2_bt_navigator::BtNavigator',
                name='bt_navigator',
                remappings=remappings,
                parameters=[
                    navigation_configured_params
                ],
            ),
            'task_manager': ComposableNode(
                package='nav2_task_manager',
                plugin='nav2_task_manager::TaskManager',
                name='task_manager',
                remappings=remappings,
                parameters=[
                    navigation_configured_params
                ],
            ),
            'sensor_filter': ComposableNode(
                package='sensor_vel_filters',
                plugin='sensor_vel_filters::SensorFilter',
                name='sensor_filter',
                remappings=remappings,
                parameters=[
                    navigation_configured_params
                ],
            ),
        }
    elif ROBOT_MODEL == "ginger_lite":
        lifecycle_managed_nodes_map = {
            'collision_monitor': ComposableNode(
                package='nav2_collision_monitor',
                plugin='nav2_collision_monitor::CollisionMonitor',
                name='collision_monitor',
                parameters=[
                    collision_monitor_configured_params
                ],
            ),
            'controller_server': ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                remappings=remappings,
                parameters=[
                    navigation_configured_params
                ],
            ),
            'planner_server': ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                remappings=remappings,
                parameters=[
                    navigation_configured_params
                ],
            ),
            'local_planner_server': ComposableNode(
                package='nav2_local_planner',
                plugin='nav2_local_planner::LocalPlannerServer',
                name='local_planner_server',
                remappings=remappings,
                parameters=[
                    navigation_configured_params
                ],
            ),
            'recoveries_server': ComposableNode(
                package='nav2_recoveries',
                plugin='recovery_server::RecoveryServer',
                name='recoveries_server',
                remappings=remappings,
                parameters=[
                    navigation_configured_params
                ],
            ),
            'bt_navigator': ComposableNode(
                package='nav2_bt_navigator',
                plugin='nav2_bt_navigator::BtNavigator',
                name='bt_navigator',
                remappings=remappings,
                parameters=[
                    navigation_configured_params
                ],
            ),
            'waypoint_follower': ComposableNode(
                package='nav2_waypoint_follower',
                plugin='nav2_waypoint_follower::WaypointFollower',
                name='waypoint_follower',
                remappings=remappings,
                parameters=[
                    navigation_configured_params
                ],
            ),
            'task_manager': ComposableNode(
                package='nav2_task_manager',
                plugin='nav2_task_manager::TaskManager',
                name='task_manager',
                remappings=remappings,
                parameters=[
                    navigation_configured_params
                ],
            ),
        }
    elif ROBOT_MODEL == "patrol":
        lifecycle_managed_nodes_map = {
            'collision_monitor': ComposableNode(
                package='nav2_collision_monitor',
                plugin='nav2_collision_monitor::CollisionMonitor',
                name='collision_monitor',
                parameters=[
                    collision_monitor_configured_params
                ],
            ),
            'controller_server': ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                remappings=remappings,
                parameters=[
                    navigation_configured_params
                ],
            ),
            'planner_server': ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                remappings=remappings,
                parameters=[
                    navigation_configured_params
                ],
            ),
            'recoveries_server': ComposableNode(
                package='nav2_recoveries',
                plugin='recovery_server::RecoveryServer',
                name='recoveries_server',
                remappings=remappings,
                parameters=[
                    navigation_configured_params
                ],
            ),
            'bt_navigator': ComposableNode(
                package='nav2_bt_navigator',
                plugin='nav2_bt_navigator::BtNavigator',
                name='bt_navigator',
                remappings=remappings,
                parameters=[
                    navigation_configured_params
                ],
            ),
            'task_manager': ComposableNode(
                package='nav2_task_manager',
                plugin='nav2_task_manager::TaskManager',
                name='task_manager',
                remappings=remappings,
                parameters=[
                    navigation_configured_params
                ],
            ),
        }
    elif ROBOT_MODEL == "simu":
        lifecycle_managed_nodes_map = {
            'collision_monitor': ComposableNode(
                package='nav2_collision_monitor',
                plugin='nav2_collision_monitor::CollisionMonitor',
                name='collision_monitor',
                parameters=[
                    collision_monitor_configured_params
                ],
            ),
            'controller_server': ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                remappings=remappings,
                parameters=[
                    navigation_configured_params
                ],
            ),
            'planner_server': ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                remappings=remappings,
                parameters=[
                    navigation_configured_params
                ],
            ),
            'recoveries_server': ComposableNode(
                package='nav2_recoveries',
                plugin='recovery_server::RecoveryServer',
                name='recoveries_server',
                remappings=remappings,
                parameters=[
                    navigation_configured_params
                ],
            ),
            'bt_navigator': ComposableNode(
                package='nav2_bt_navigator',
                plugin='nav2_bt_navigator::BtNavigator',
                name='bt_navigator',
                remappings=remappings,
                parameters=[
                    navigation_configured_params
                ],
            ),
            'task_manager': ComposableNode(
                package='nav2_task_manager',
                plugin='nav2_task_manager::TaskManager',
                name='task_manager',
                remappings=remappings,
                parameters=[
                    navigation_configured_params
                ],
            ),
            'sensor_filter': ComposableNode(
                package='sensor_vel_filters',
                plugin='sensor_vel_filters::SensorFilter',
                name='sensor_filter',
                remappings=remappings,
                parameters=[
                    navigation_configured_params
                ],
            ),
        }
    else:
        lifecycle_managed_nodes_map = {
            'collision_monitor': ComposableNode(
                package='nav2_collision_monitor',
                plugin='nav2_collision_monitor::CollisionMonitor',
                name='collision_monitor',
                parameters=[
                    collision_monitor_configured_params
                ],
            ),
            'controller_server': ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                remappings=remappings,
                parameters=[
                    navigation_configured_params
                ],
            ),
            'planner_server': ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                remappings=remappings,
                parameters=[
                    navigation_configured_params
                ],
            ),
            'local_planner_server': ComposableNode(
                package='nav2_local_planner',
                plugin='nav2_local_planner::LocalPlannerServer',
                name='local_planner_server',
                remappings=remappings,
                parameters=[
                    navigation_configured_params
                ],
            ),
            'recoveries_server': ComposableNode(
                package='nav2_recoveries',
                plugin='recovery_server::RecoveryServer',
                name='recoveries_server',
                remappings=remappings,
                parameters=[
                    navigation_configured_params
                ],
            ),
            'bt_navigator': ComposableNode(
                package='nav2_bt_navigator',
                plugin='nav2_bt_navigator::BtNavigator',
                name='bt_navigator',
                remappings=remappings,
                parameters=[
                    navigation_configured_params
                ],
            ),
            'waypoint_follower': ComposableNode(
                package='nav2_waypoint_follower',
                plugin='nav2_waypoint_follower::WaypointFollower',
                name='waypoint_follower',
                remappings=remappings,
                parameters=[
                    navigation_configured_params
                ],
            ),
            'task_manager': ComposableNode(
                package='nav2_task_manager',
                plugin='nav2_task_manager::TaskManager',
                name='task_manager',
                remappings=remappings,
                parameters=[
                    navigation_configured_params
                ],
            ),
        }

    for lifecycle_managed_node_name, value in lifecycle_managed_nodes_map.items():
        print("Loading rclcpp component %s for lifecycle managed node \'%s\'" % (value.node_plugin[0].describe(), lifecycle_managed_node_name))

    container = ComposableNodeContainer(
        name='smart_pilot_system',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=list(lifecycle_managed_nodes_map.values()) + [
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_navigation',
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': list(lifecycle_managed_nodes_map.keys())}
                ]
            )
        ],
        output='screen',
        respawn=True,
    )

    ld = launch.LaunchDescription()

    # Launch arguments
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_navi_params_file_cmd)
    ld.add_action(declare_collision_params_file_cmd)

    # Node launching commands
    ld.add_action(container)
    return ld
