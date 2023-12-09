#! /usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from waypoint_nav2.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
import time

def main():
    rclpy.init()
    navigator = BasicNavigator()
    
    navigator.waitWaypoints()
    # print(navigator.waypointArray)
    
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = -0.19006443
    initial_pose.pose.position.y = -11.14016
    initial_pose.pose.orientation.z = -0.00829
    initial_pose.pose.orientation.w = 0.99996563



    navigator.setInitialPose(initial_pose)

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active()


    # set our demo's goal poses to follow
    goal_poses = navigator.make_goal_pose(initial_pose)


    nav_start = navigator.get_clock().now()
    navigator.followWaypoints(goal_poses)

    i = 0
    while not navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Executing current waypoint: ' +
                  str(feedback.current_waypoint + 1) + '/' + str(len(goal_poses)))
            now = navigator.get_clock().now()

            # Some navigation timeout to demo cancellation
            if now - nav_start > Duration(seconds=10000.0):
                navigator.cancelTask()


    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()

    # position:
    #   x: 0.24913541972637177
    #   y: -0.14622102677822113
    #   z: 0.0
    # orientation:
    #   x: 0.0
    #   y: 0.0
    #   z: -0.0014584504188100763
    #   w: 0.9999989364606223
