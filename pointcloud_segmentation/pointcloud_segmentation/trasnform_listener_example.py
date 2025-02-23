import rclpy
from rclpy.node import Node

import tf2_ros
import tf_transformations

import open3d as o3d
import numpy as np

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

def visualize_point_cloud(pcd):
    # Create a coordinate frame
    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
    size=1.0,      # Length of the axes
    origin=[0, 0, 0]  # Origin point
    )
    # Assuming 'pcd' is your PointCloud object
    o3d.visualization.draw_geometries(
    [pcd, coordinate_frame],
    window_name='Point Cloud with Coordinate Frame',
    width=800,
    height=600,
    left=50,
    top=50,
    point_show_normal=False
)

class RealTimePointCloudTransformer(Node):
    def __init__(self):
        super().__init__('real_time_pcd_transformer')

        # Initialize TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscribe to PointCloud2 topic
        self.subscription = self.create_subscription(
            PointCloud2,
            '/points2',  # Replace with your actual topic
            self.point_cloud_callback,
            10
        )

    def point_cloud_callback(self, msg):
        # Extract points from PointCloud2 message
        points = []
        for point in point_cloud2.read_points(
            msg,
            field_names=['x', 'y', 'z'],
            skip_nans=True
        ):
            points.append([point[0], point[1], point[2]])

        if not points:
            self.get_logger().warn("No valid points found in PointCloud2 message.")
            return

        # Convert to NumPy array
        points_np = np.array(points, dtype=np.float32)

        # Create Open3D point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_np)

        # Optionally assign colors
        pcd.paint_uniform_color([0.0, 1.0, 0.0])  # Green

        # Get the transform from /rgb_camera_link to /map
        try:
            transform_stamped = self.tf_buffer.lookup_transform(
                'map',
                'rgb_camera_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
        except Exception as e:
            self.get_logger().error(f"Transform unavailable: {e}")
            return

        # Convert transform to 4x4 matrix
        transformation_matrix = self.transform_stamped_to_matrix(transform_stamped)

        # Apply transformation
        pcd.transform(transformation_matrix)

        # Visualize the transformed point cloud
        # Note: Visualization inside a callback can block; consider threading or alternative approaches
        visualize_point_cloud(pcd)

    def transform_stamped_to_matrix(self, transform_stamped):
        # Extract translation
        translation = transform_stamped.transform.translation
        translation_matrix = tf_transformations.translation_matrix(
            (translation.x, translation.y, translation.z)
        )

        # Extract rotation (quaternion)
        rotation = transform_stamped.transform.rotation
        quaternion = (rotation.x, rotation.y, rotation.z, rotation.w)
        rotation_matrix = tf_transformations.quaternion_matrix(quaternion)

        # Combine into a 4x4 matrix
        transformation_matrix = np.dot(translation_matrix, rotation_matrix)

        return transformation_matrix

def main(args=None):
    rclpy.init(args=args)
    transformer = RealTimePointCloudTransformer()
    rclpy.spin(transformer)
    transformer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
