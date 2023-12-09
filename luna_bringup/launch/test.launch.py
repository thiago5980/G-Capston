import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    Node(
      package='test_py_cpp_1',
      executable='helloworld_publisher',
      name='helloworld_publisher',
      output='screen'),
    Node(
      package='test_py_cpp_1',
      executable='helloworld_subscriber.py',
      name='helloworld_subscriber',
      output='screen',
      )
])