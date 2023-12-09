#!/usr/bin/env python3
import rclpy
from rclpy.qos import QoSProfile
from rclpy.node import Node

# import tf2
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Quaternion, TransformStamped, PoseWithCovariance, TwistWithCovariance, Vector3, Twist

from sensor_msgs.msg import Imu
from luna_msg.msg import Pose
import math


def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr
    return q 

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')
        self.publisher_ = self.create_publisher(Odometry, 'odom', 10)
        # self.publisher_odom = self.create_publisher(Odometry, 'odom', 10)
        self.subscription = self.create_subscription(Pose,'robot_pose',self.callback,10)
        # self.subscription_odom = self.create_subscription(Odometry,'odometry/filtered',self.odom_callback,10)
        self.subscription_odom = self.create_subscription(Imu,'imu/data',self.imu_callback,10)
        self.pub_wheel_OdomTF = TransformBroadcaster(self,qos=QoSProfile(depth=100))
        self._imu = Quaternion()
        self._imu_twist = TwistWithCovariance()
        # self.pub_OdomTF = TransformBroadcaster(self)
    # def publish_odometry(self, x, y, z, quat_x, quat_y, quat_z, quat_w):
    #     msg = Odometry()
    #     msg.header.stamp = self.get_clock().now().to_msg()
    #     msg.pose.pose.position.x = x
    #     print('pos x:', x, ' y:', y)
    #     msg.pose.pose.position.y = y
    #     msg.pose.pose.position.z = z
    #     msg.pose.pose.orientation.x = quat_x
    #     msg.pose.pose.orientation.y = quat_y    
    #     msg.pose.pose.orientation.z = quat_z
    #     msg.pose.pose.orientation.w = quat_w
    #     self.publisher_.publish(msg)
    
    def imu_callback(self, msg):
        self._imu = msg.orientation
        
        self._imu_twist.twist.angular = msg.angular_velocity
        for i in range(3):
            for j in range(3):
                self._imu_twist.covariance[i*6+j] = msg.angular_velocity_covariance[i*3+j]

        
    def callback(self, robot_pose):
        q = quaternion_from_euler(0, 0, robot_pose.theta)
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_footprint"
        msg.pose.pose.position.x = robot_pose.x
        msg.pose.pose.position.y = robot_pose.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = q
        # msg.pose.pose.orientation = self._imu
        # msg.twist = self._imu_twist
        msg.twist.twist.linear.x = robot_pose.linear_velocity
        msg.twist.twist.angular.z = robot_pose.angular_velocity
        # msg.twist.twist.angular = self._imu_twist.twist.angular
        self.publisher_.publish(msg)
        odom_tf = TransformStamped()
        odom_tf.header.frame_id = msg.header.frame_id
        odom_tf.child_frame_id = msg.child_frame_id
        odom_tf.header.stamp = self.get_clock().now().to_msg()

        odom_tf.transform.translation.x = msg.pose.pose.position.x
        odom_tf.transform.translation.y = msg.pose.pose.position.y
        odom_tf.transform.translation.z = msg.pose.pose.position.z
        odom_tf.transform.rotation = msg.pose.pose.orientation
        self.pub_wheel_OdomTF.sendTransform(odom_tf)

        

def main(args=None):
    print('------------------')
    rclpy.init(args=args)
    node = OdometryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()