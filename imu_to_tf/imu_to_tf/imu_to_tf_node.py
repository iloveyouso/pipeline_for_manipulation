# imu_to_tf_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import tf_transformations
import tf2_ros
from geometry_msgs.msg import TransformStamped
import numpy as np

class IMUToTFNode(Node):
    def __init__(self):
        super().__init__('imu_to_tf_node')
        self.subscription = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.get_logger().info('IMU to TF node has been started.')

    def imu_callback(self, msg):
        # Extract linear acceleration
        a_x = msg.linear_acceleration.x
        a_y = msg.linear_acceleration.y
        a_z = msg.linear_acceleration.z

        # Normalize the acceleration vector to get the gravity direction
        norm = np.linalg.norm([a_x, a_y, a_z])
        if norm == 0:
            self.get_logger().warn('Received zero acceleration vector.')
            return
        a_x_norm = -a_x / norm
        a_y_norm = -a_y / norm
        a_z_norm = -a_z / norm

        # Compute roll and pitch from the acceleration
        roll = -np.arctan2(a_y_norm, a_z_norm)
        pitch = np.arctan2(-a_x_norm, np.sqrt(a_y_norm**2 + a_z_norm**2))
        yaw = 0.0  # Cannot be determined from accelerometer alone

        # Convert Euler angles to quaternion
        q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)

        # Create and populate the TransformStamped message
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'camera_base_parallel_floor'  # Base frame
        t.child_frame_id = 'camera_base'  # Camera frame

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Broadcast the transformation
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = IMUToTFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
