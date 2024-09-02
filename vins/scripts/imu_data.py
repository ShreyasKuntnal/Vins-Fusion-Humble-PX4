#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from px4_msgs.msg import SensorCombined, VehicleAttitude

from sensor_msgs.msg import Image

# Configure QoS profile for publishing and subscribing
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# QoS profile for publishing IMU data
imu_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

# QoS profile for subscribing to SensorCombined and VehicleAttitude
sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=5
)

global latest_attitude

rclpy.init()
try:
    print("RClpy initialized")
    node = Node("SensorToImuConverter")
except Exception as e:
    print(f"Error initializing SensorToImuConverter: {e}")
    # return

# class SensorToImuConverter(Node):
#     def __init__(self):
#         super().__init__('SensorToImuConverter')


def vehicle_attitude_callback(msg1):
    node.get_logger().info("Vehicle attitude callback")
    latest_attitude = msg1.q
    print("Latest attitude: ", msg1)

def sensor_combined_callback(msg2):
    # node.get_logger().info('Processing sensor combined data...')

    # print("Sensor combined callback: ", msg2)
    imu_msg = Imu()
    
    # Set the header
    imu_msg.header = Header()
    imu_msg.header.stamp = node.get_clock().now().to_msg()
    imu_msg.header.frame_id = 'camera_imu_optical_frame'

    # print("Latest attitude: ", latest_attitude)
    # Set orientation from vehicle attitude if available
    # if latest_attitude in globals() and latest_attitude is not None:
    imu_msg.orientation.w = float(latest_attitude.q[0])
    imu_msg.orientation.x = float(latest_attitude.q[1])
    imu_msg.orientation.y = float(latest_attitude.q[2])
    imu_msg.orientation.z = float(latest_attitude.q[3])
    # else:
    #     node.get_logger().info("No vehicle attitude data available")
    #     imu_msg.orientation.x = 0.0
    #     imu_msg.orientation.y = 0.0
    #     imu_msg.orientation.z = 0.0
    #     imu_msg.orientation.w = 0.0

    # Set orientation covariance to -1 (unknown)
    imu_msg.orientation_covariance = [-1.0] + [0.0] * 8
    # print("Orientation: ", msg2)
    # Set angular velocity
    imu_msg.angular_velocity.x = float(msg2.gyro_rad[0])
    imu_msg.angular_velocity.y = float(msg2.gyro_rad[1])
    imu_msg.angular_velocity.z = float(msg2.gyro_rad[2])

    # Set angular velocity covariance (assuming 0.01 for all axes)
    imu_msg.angular_velocity_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]

    
    # Set linear acceleration
    imu_msg.linear_acceleration.x = float(msg2.accelerometer_m_s2[0])
    imu_msg.linear_acceleration.y = float(msg2.accelerometer_m_s2[1])
    imu_msg.linear_acceleration.z = float(msg2.accelerometer_m_s2[2])

    # Set linear acceleration covariance (assuming 0.01 for all axes)
    imu_msg.linear_acceleration_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]

    publisher.publish(imu_msg)


publisher = node.create_publisher(Imu, '/imu/data', imu_qos)
sensor_combined_subscription = node.create_subscription(
    SensorCombined, '/fmu/out/sensor_combined', sensor_combined_callback, sensor_qos)
vehicle_attitude_subscription = node.create_subscription(
    VehicleAttitude, '/fmu/out/vehicle_attitude', vehicle_attitude_callback, sensor_qos)
latest_attitude = None

# def main(args=None):

try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass

# Clean up and shutdown
node.destroy_node()
rclpy.shutdown()