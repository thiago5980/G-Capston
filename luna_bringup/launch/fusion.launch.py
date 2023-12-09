import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir

def generate_launch_description():
    robot_localization_file_path = os.path.join(get_package_share_directory('luna_bringup'), 'config','ekf.yaml')
    # use_sim_time = LaunchConfiguration('use_sim_time')


    return LaunchDescription([
        # DeclareLaunchArgument(
        #     name='use_sim_time',
        #     default_value='True',
        #     description='Use simulation clock if true'),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[robot_localization_file_path],
        ),
        # Node(
        # package='rviz2',
        # executable='rviz2',
        # name='rviz2',
        # arguments=['-d', rviz_config_file],
        # )
        
])