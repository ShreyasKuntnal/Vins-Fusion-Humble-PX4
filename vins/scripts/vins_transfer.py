#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry
import math
from pyquaternion import Quaternion
import tf_transformations
import sys

# class vinsTransfer(Node):
#     """Node for tranferring vins data"""

#     def __init__(self) -> None:
#         super().__init__('vins_transfer')

#         self.local_pose = PoseStamped()
#         self.local_pose.header.frame_id = 'world'
#         quaternion = tf_transformations.quaternion_from_euler(0, -math.pi/2, math.pi/2)
#         q = Quaternion([quaternion[3],quaternion[0],quaternion[1],quaternion[2]])
#         self.q=q

#         # Configure QoS profile for publishing and subscribing
#         qos_profile = QoSProfile(
#             reliability=ReliabilityPolicy.BEST_EFFORT,
#             durability=DurabilityPolicy.VOLATILE,
#             history=HistoryPolicy.KEEP_LAST,
#             depth=1
#         )

#         # Create subscribers
#         self.subscription = self.create_subscription(
#             Odometry, '/camera_pose', self.vins_callback, qos_profile)

#         # Create publishers
#         self.position_pub = self.create_publisher(
#             PoseStamped, '/fmu/in/vehicle_visual_odometry', qos_profile)

#         self.rate = self.create_rate(30)

#     def vins_callback(self, data):
#         self.local_pose.pose.position.x = data.pose.pose.position.x
#         self.local_pose.pose.position.y = data.pose.pose.position.y
#         self.local_pose.pose.position.z = data.pose.pose.position.z
#         q_= Quaternion([data.pose.pose.orientation.w,data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z])
#         q_ = q_*self.q
#         self.local_pose.pose.orientation.w = q_[0]
#         self.local_pose.pose.orientation.x = q_[1]
#         self.local_pose.pose.orientation.y = q_[2]
#         self.local_pose.pose.orientation.z = q_[3]
#     def run(self):
#         while rclpy.ok():
#             print("We are into while loop")
#             if self.local_pose.pose.position == Point():
#                 print("I will continue")
#                 continue
#             else:
#                 print("We are into esle condition")
#                 self.get_logger().info("Vins pose received")
#                 self.local_pose.header.stamp = self.get_clock().now().to_msg()
#                 print(f"I am here to publish this:{self.local_pose}")
#                 self.position_pub.publish(self.local_pose)

#             try:
#                 self.rate.sleep()
#             except KeyboardInterrupt:
#                 break


# def main(args=None) -> None:
#     print('Starting offboard control node...')
#     rclpy.init(args=args)
#     vins_transfer = vinsTransfer()
#     try:
#         vins_transfer.run()
#         # rclpy.spin(vins_transfer)
#     except (KeyboardInterrupt, ExternalShutdownException):
#         pass
#     finally:
#         vins_transfer.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     try:
#         main()
#     except Exception as e:
#         print(e)


local_pose = PoseStamped()
local_pose.header.frame_id = 'world'
quaternion = tf_transformations.quaternion_from_euler(0, -math.pi/2, math.pi/2)
q = Quaternion([quaternion[3],quaternion[0],quaternion[1],quaternion[2]])

# Configure QoS profile for publishing and subscribing
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

def vins_callback(data):
        local_pose.pose.position.x = data.pose.pose.position.x
        local_pose.pose.position.y = data.pose.pose.position.y
        local_pose.pose.position.z = data.pose.pose.position.z
        q_= Quaternion([data.pose.pose.orientation.w,data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z])
        q_ = q_*q
        local_pose.pose.orientation.w = q_[0]
        local_pose.pose.orientation.x = q_[1]
        local_pose.pose.orientation.y = q_[2]
        local_pose.pose.orientation.z = q_[3]


rclpy.init()
node=Node("vins_transfer")
subscription = node.create_subscription(
            Odometry, '/camera_pose', vins_callback, qos_profile)

        # Create publishers
position_pub = node.create_publisher(
            PoseStamped, '/fmu/in/vehicle_visual_odometry', qos_profile)

rate = node.create_rate(30)

def timer_callback():
    global local_pose
    if local_pose.pose.position == Point():
        print("We are returning")
        return
    else:
        print("We are into else")
        node.get_logger().info("Vins pose received")
        local_pose.header.stamp = node.get_clock().now().to_msg()
        position_pub.publish(local_pose)

# Create a timer to call the timer_callback at 30 Hz
timer = node.create_timer(1.0 / 30.0, timer_callback)

# Spin the node to keep it active
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass

# Clean up and shutdown
node.destroy_node()
rclpy.shutdown()