#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry
# from px4_msgs.msg import VehicleOdometry
import math
from pyquaternion import Quaternion
import tf_transformations
import sys

vehicle_type = sys.argv[1]
vehicle_id = sys.argv[2]
local_pose = PoseStamped()
local_pose.header.frame_id = 'world'
quaternion = tf_transformations.quaternion_from_euler(0, -math.pi/2, math.pi/2)
q = Quaternion([quaternion[3],quaternion[0],quaternion[1],quaternion[2]])

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
 

def main(args=None):
    try:
        rclpy.init(args=args)
        node = Node('vins_transfer')
        subscription = node.create_subscription("/camera_pose", Odometry, vins_callback,10)
        subscription  # prevent unused variable warning
        position_pub = node.create_publisher("/fmu/in/vehicle_visual_odometry", PoseStamped, 10)
        rate = node.create_rate(30)
        while rclpy.ok():
            if local_pose.pose.position == Point():
                continue
            else:
                node.get_logger().info("Vins pose received")
                local_pose.header.stamp = node.get_clock().now().to_msg()
                position_pub.publish(local_pose)

            try:
                rate.sleep()
            except KeyboardInterrupt:
                break
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        

if __name__ == '__main__':
    main()


