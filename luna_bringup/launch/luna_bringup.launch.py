import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir

def generate_launch_description():
    pkg_md = get_package_share_directory('md')
    pkg_luna_description = get_package_share_directory('luna_description')
    pkg_iahrs_driver = get_package_share_directory('iahrs_driver')
    pkg_luna_bringup = get_package_share_directory('luna_bringup')
    robot_localization_file_path = os.path.join(get_package_share_directory('luna_bringup'), 'config','ekf.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')

    rviz_config_file = os.path.join(get_package_share_directory('luna_bringup'), 'config','luna_rviz.rviz')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/luna_lidar.launch.py']),
        ),
        # TimerAction(
        #     actions=[
        #         IncludeLaunchDescription(
        #             PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/fusion.launch.py'])
        #         ),
        #     ],
        #     period=10.0  # Time in seconds after which to run the action
        # ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_md, 'launch', 'md.launch.py')
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_iahrs_driver, 'launch', 'iahrs_driver.launch.py')
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_luna_description, 'launch', 'luna_state_publisher.launch.py')
            )
        ),

        Node(
        package='luna_bringup',
        executable='luna_odom.py',
        name='luna_odom',
        output='screen',
        ),
        Node(
        package='capston_motor',
        executable='capston_mobile_motor_node',
        name='capston_mobile_motor_node',
        output='screen',
        ),
        # Node(
        # package='luna_bringup',
        # executable='filter_odom.py',
        # name='luna_filter_odom',
        # output='screen',
        # ),
        # Node(
        # package='rviz2',
        # executable='rviz2',
        # name='rviz2',
        # arguments=['-d', rviz_config_file],
        # )
        # DeclareLaunchArgument(
        #     name='use_sim_time',
        #     default_value='True',
        #     description='Use simulation clock if true'),
])