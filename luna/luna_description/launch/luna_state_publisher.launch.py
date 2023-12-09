#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch import condition
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  use_sim_time = LaunchConfiguration('use_sim_time', default='false')
  urdf_file_name = 'luna.urdf'

  print("urdf_file_name : {}".format(urdf_file_name))
  
  urdf = os.path.join(
    get_package_share_directory('luna_description'),
    'urdf',
    urdf_file_name)
  
  with open(urdf, 'r') as infp:
    robot_desc = infp.read()
  rsp_params = {'robot_description': robot_desc}  
  
  return LaunchDescription([
    DeclareLaunchArgument(
      'use_sim_time',
      default_value='false',
      description='Use simulation (Gazebo) clock if true'),
    Node(
      package='robot_state_publisher',
      executable='robot_state_publisher',
      name='robot_state_publisher',
      output='screen',
      parameters=[rsp_params, {'use_sim_time':use_sim_time}]),
    # Node(
    #   package='joint_state_publisher',
    #   executable='joint_state_publisher',
    #   name='joint_state_publisher',
    #   # condition=IfCondition(LaunchConfiguration("publish_joints"))
    # )
  ])
  