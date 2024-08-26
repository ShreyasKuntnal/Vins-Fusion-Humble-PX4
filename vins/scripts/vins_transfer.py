#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
from tf_transformations import quaternion_multiply

import numpy as np
import math
from pyquaternion import Quaternion
import tf_transformations
import sys

vehicle_odometry_msg=VehicleOdometry()
quaternion = tf_transformations.quaternion_from_euler(0, -math.pi/2, math.pi/2)
# q = Quaternion([quaternion[3],quaternion[0],quaternion[1],quaternion[2]])
q = Quaternion(w=quaternion[3], x=quaternion[0], y=quaternion[1], z=quaternion[2])

# Configure QoS profile for publishing and subscribing
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

def vins_callback(data):
        if data.pose.pose.position == Point():
            print(data.pose.pose.position)
            print("We are returning as position recieved is 0s")
            return
        else:
            # print("We are into else")
            node.get_logger().info("Vins pose received")
            # Fill in the position and orientation
            # vehicle_odometry_msg.position[0] = data.pose.pose.position.x
            # vehicle_odometry_msg.position[1] = data.pose.pose.position.y
            # vehicle_odometry_msg.position[2] = data.pose.pose.position.z

            # Fill in the position (NED frame)
            vehicle_odometry_msg.x = data.pose.pose.position.x
            vehicle_odometry_msg.y = data.pose.pose.position.y
            vehicle_odometry_msg.z = data.pose.pose.position.z
            # q_ = Quaternion([data.pose.pose.orientation.w, data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z])
            # q_ = q_ * q
            # Adjust orientation using the pre-defined quaternion
            q_data = [data.pose.pose.orientation.x, data.pose.pose.orientation.y,
                    data.pose.pose.orientation.z, data.pose.pose.orientation.w]
            q_ = quaternion_multiply([q.x, q.y, q.z, q.w], q_data)
            # vehicle_odometry_msg.q[0] = q_[0]
            # vehicle_odometry_msg.q[1] = q_[1]
            # vehicle_odometry_msg.q[2] = q_[2]
            # vehicle_odometry_msg.q[3] = q_[3]
            vehicle_odometry_msg.q[0] = q_[3]  # w
            vehicle_odometry_msg.q[1] = q_[0]  # x
            vehicle_odometry_msg.q[2] = q_[1]  # y
            vehicle_odometry_msg.q[3] = q_[2]  # z

            # vehicle_odometry_msg.pose_frame = VehicleOdometry.POSE_FRAME_NED

            # vehicle_odometry_msg.timestamp = node.get_clock().now().nanoseconds // 1000  # Convert to microseconds
            # vehicle_odometry_msg.timestamp_sample = data.header.stamp.sec * 1_000_000 + data.header.stamp.nanosec // 1000

            # Set pose frame to NED
            vehicle_odometry_msg.local_frame = VehicleOdometry.LOCAL_FRAME_NED

            # Set timestamp and sample timestamp
            vehicle_odometry_msg.timestamp = node.get_clock().now().nanoseconds // 1000  # Convert to microseconds
            vehicle_odometry_msg.timestamp_sample = data.header.stamp.sec * 1_000_000 + data.header.stamp.nanosec // 1000

            # print(f"We are publishing :{vehicle_odometry_msg} ")
            position_pub.publish(vehicle_odometry_msg)


rclpy.init()
node=Node("vins_transfer")
subscription = node.create_subscription(
           Odometry, '/camera_pose', vins_callback, qos_profile)

        # Create publishers
position_pub = node.create_publisher(
            VehicleOdometry, '/fmu/in/vehicle_visual_odometry', qos_profile)

rate = node.create_rate(30)

# Spin the node to keep it active
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass

# Clean up and shutdown
node.destroy_node()
rclpy.shutdown()
