#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import TransformStamped
import tf2_ros

class StaticFramePublisher(Node):
    def __init__(self):
        super().__init__('static_frame_publisher')
        self.broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # Create a TransformStamped message
        static_transformStamped = TransformStamped()
        static_transformStamped.header.stamp = self.get_clock().now().to_msg()
        static_transformStamped.header.frame_id = 'world'  # Parent frame
        static_transformStamped.child_frame_id = 'camera_base_parallel_floor'  # Child frame

        # --- Translation parameters (in meters) ---
        # Set your x, y, z translation values here.
        static_transformStamped.transform.translation.x = 0.20 # Example: 1.0 meter along x
        static_transformStamped.transform.translation.y = 0.025  # Example: 2.0 meters along y
        static_transformStamped.transform.translation.z = 0.13  # Example: 0.5 meter along z
        # self.get_logger().info(f'x,y,z: {static_transformStamped.transform.translation.x},{static_transformStamped.transform.translation.y},{static_transformStamped.transform.translation.z}")

        # --- Rotation parameters (in degrees) ---
        # Set your roll (rx), pitch (ry), and yaw (rz) angles.
        # These values will be converted to a quaternion.
        roll  = math.radians(0.0)   # Replace 0.0 with your roll value
        pitch = math.radians(0.0)   # Replace 0.0 with your pitch value
        yaw   = math.radians(0.0)   # Replace 0.0 with your yaw value

        qx, qy, qz, qw = self.euler_to_quaternion(roll, pitch, yaw)
        static_transformStamped.transform.rotation.x = qx
        static_transformStamped.transform.rotation.y = qy
        static_transformStamped.transform.rotation.z = qz
        static_transformStamped.transform.rotation.w = qw

        # Publish the static transform once.
        self.broadcaster.sendTransform(static_transformStamped)
        self.get_logger().info("Static transform published from '/world' to '/camera_base_parallel_floor'")

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion."""
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return qx, qy, qz, qw

def main(args=None):
    rclpy.init(args=args)
    node = StaticFramePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
