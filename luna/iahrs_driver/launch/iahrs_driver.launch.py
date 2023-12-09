#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.conditions import IfCondition
from launch import condition
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  return LaunchDescription([
    ExecuteProcess(
      cmd=["ros2","run","iahrs_driver","iahrs_driver"]
    ),
  ])