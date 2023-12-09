from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import LogInfo

import lifecycle_msgs.msg
import os


def generate_launch_description():
  robot_states_param = LaunchConfiguration(
    'robot_states_param',
    default=os.path.join(
      get_package_share_directory('capston_main'),
      'params',
      'make_plan.yaml'))

  return LaunchDescription([
    DeclareLaunchArgument(
      'robot_states_param',
      default_value=robot_states_param
    ),
        
    Node(
        package='capston_main',
        executable='capston_main_node',
        name='capston_main_node',
        parameters=[robot_states_param],
        output='screen'
    ),
    # Node(
    #     package='capston_motor',
    #     executable='capston_mobile_motor_node',
    #     name='capston_mobile_motor_node',
    #     output='screen'
    # ),
    Node(
        package='capston_plan',
        executable='planner',
        name='coverage_pathplanner_node',
        output='screen'
    ),
    Node(
        package='waypoint_nav2',
        executable='waypoint_follower',
        name='basic_navigator',
        output='screen'
    ),

  ])