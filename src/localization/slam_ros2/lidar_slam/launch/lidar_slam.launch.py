# launch 
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

# launch
def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value = 'False')
    param_dir = os.path.join(
        get_package_share_directory('lidar_slam'),
        'params',
        'slam.yaml'
    )

    return LaunchDescription([
        use_sim_time_arg,
        Node(
            package='lidar_slam',
            name='lidar_slam_node',
            executable='lidar_slam_node',
            parameters=[param_dir],
            arguments = [
                '-configuration_cloud_directory', '/home/ginger/hari_dyna_params',
                '-configuration_cloud_basename', 'cloud_cartographer.lua',    
                '-configuration_directory', FindPackageShare('lidar_slam').find('lidar_slam') + '/params/cartographer',
                '-configuration_basename', 'cartographer.lua'],
            # arguments = [
            #     '-configuration_directory', FindPackageShare('cartographer_ros').find('cartographer_ros') + '/configuration_files',
            #     '-configuration_basename', 'backpack_2d.lua'],
            output='screen'
        )
    ])