import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

# 定义函数名称为：generate_launch_description
def generate_launch_description():
     # Environment
    # Launch arguments
    # 1. Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')

    # 2. Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {}

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    start_params_manager_cmd = Node(
        package="params_manager",
        executable="params_manager",
        namespace='cloudminds',
        output='log',
        parameters=[configured_params]
        )

    # 创建LaunchDescription对象launch_description,用于描述launch文件
    ld = LaunchDescription()
    # 返回让ROS2根据launch描述执行节点
    ld.add_action(declare_namespace_cmd)

    ld.add_action(start_params_manager_cmd)
    return ld
