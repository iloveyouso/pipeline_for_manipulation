# pointcloud_segmentation/segmentation_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import open3d as o3d
from sensor_msgs_py import point_cloud2
import struct
import threading

import tf2_ros
import tf_transformations

def visualize_point_cloud(pcd, cuboid_center, cuboid_dimensions):
    """
    Visualize the given Open3D point cloud with a coordinate frame and cuboid boundaries.
    """
    geometries = []

    # Add the filtered point cloud
    geometries.append(pcd)

    # Create a cuboid (Axis-Aligned Bounding Box)
    min_bound = [
        cuboid_center[0] - cuboid_dimensions[0] / 2,
        cuboid_center[1] - cuboid_dimensions[1] / 2,
        cuboid_center[2] - cuboid_dimensions[2] / 2
    ]
    max_bound = [
        cuboid_center[0] + cuboid_dimensions[0] / 2,
        cuboid_center[1] + cuboid_dimensions[1] / 2,
        cuboid_center[2] + cuboid_dimensions[2] / 2
    ]
    aabb = o3d.geometry.AxisAlignedBoundingBox(min_bound=min_bound, max_bound=max_bound)
    aabb.color = (1, 0, 0)  # Red color for the cuboid

    geometries.append(aabb)

    # Add a coordinate frame for reference
    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=1.0,      # Length of the axes
        origin=[0, 0, 0]  # Origin point
    )
    geometries.append(coordinate_frame)

    # Visualize all geometries together
    o3d.visualization.draw_geometries(
        geometries,
        window_name='Filtered Point Cloud with Cuboid',
        width=800,
        height=600,
        left=50,
        top=50,
        point_show_normal=False
    )

def pointcloud2_to_o3d(pc2_msg):
    """
    Convert a ROS PointCloud2 message to an Open3D PointCloud object.
    """
    # Extract field names
    field_names = [field.name for field in pc2_msg.fields]

    # Read points from PointCloud2 message
    points = []
    for point in point_cloud2.read_points(pc2_msg, field_names=field_names, skip_nans=True):
        # Extract x, y, z and append as a sublist
        points.append([point['x'], point['y'], point['z']])

    # Convert the list of lists to a 2D NumPy array
    points = np.array(points, dtype=np.float32)

    if points.size == 0:
        return None

    # Create Open3D point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    return pcd

def o3d_to_pointcloud2(cloud, header):
    """
    Convert an Open3D PointCloud object to a ROS PointCloud2 message.
    """
    points = np.asarray(cloud.points)
    if points.size == 0:
        return None

    # Ensure the points are in float32 format
    points = points.astype(np.float32)

    # Use create_cloud_xyz32 to create the PointCloud2 message
    msg = point_cloud2.create_cloud_xyz32(header, points)

    return msg

class SegmentationNode(Node):
    def __init__(self):
        super().__init__('pointcloud_segmentation_node')

        # Declare parameters for cuboid dimensions and center
        self.declare_parameter('cuboid_center_x', 0.3)
        self.declare_parameter('cuboid_center_y', 0.0)
        self.declare_parameter('cuboid_center_z', 0.0)
        self.declare_parameter('cuboid_length_x', 0.3)
        self.declare_parameter('cuboid_length_y', 0.3)
        self.declare_parameter('cuboid_length_z', 0.30) # HEURISTIC: Increase Z dimension to include the top surface
        self.declare_parameter('visualize', False)

        # Retrieve parameter values
        self.cuboid_center_x = self.get_parameter('cuboid_center_x').get_parameter_value().double_value
        self.cuboid_center_y = self.get_parameter('cuboid_center_y').get_parameter_value().double_value
        self.cuboid_center_z = self.get_parameter('cuboid_center_z').get_parameter_value().double_value
        self.cuboid_length_x = self.get_parameter('cuboid_length_x').get_parameter_value().double_value
        self.cuboid_length_y = self.get_parameter('cuboid_length_y').get_parameter_value().double_value
        self.cuboid_length_z = self.get_parameter('cuboid_length_z').get_parameter_value().double_value
        self.visualize = self.get_parameter('visualize').get_parameter_value().bool_value


        # Validate cuboid dimensions
        if self.cuboid_length_x <= 0 or self.cuboid_length_y <= 0 or self.cuboid_length_z <= 0:
            self.get_logger().error('Cuboid dimensions must be positive.')
            raise ValueError('Invalid cuboid dimensions.')

        self.get_logger().info(f"Cuboid Center: ({self.cuboid_center_x}, {self.cuboid_center_y}, {self.cuboid_center_z})")
        self.get_logger().info(f"Cuboid Dimensions: X={self.cuboid_length_x}, Y={self.cuboid_length_y}, Z={self.cuboid_length_z}")

        # Initialize TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscription to input PointCloud2
        self.subscription = self.create_subscription(
            PointCloud2,
            '/points2',  # Replace with your input topic
            self.pointcloud_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Publisher for filtered point cloud
        self.filtered_publisher = self.create_publisher(PointCloud2, '/filtered_points', 10)

        self.get_logger().info('PointCloud Segmentation Node Initialized.')

        # To handle visualization in a separate thread
        self.visualization_thread = None
        self.visualization_lock = threading.Lock()

    def pointcloud_callback(self, msg):
        self.get_logger().info('Received PointCloud2 message.')

        # Transform the point cloud to the 'camera_base_parallel_floor' frame
        try:
            transform = self.tf_buffer.lookup_transform(
                'camera_base_parallel_floor',  # target frame
                msg.header.frame_id,  # source frame
                rclpy.time.Time(),  # time
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
        ### BOTTLENECK: Transforming the point cloud using Open3D ###
            self.get_logger().info('transform found') # from rgb_camera_link to camera_base_parallel_floor
            pcd = self.transform_pointcloud(msg, transform) # approximately <2.0s
            self.get_logger().info('transformed')
        ###########################################
        except Exception as e:
            self.get_logger().error(f"Transform unavailable: {e}")
            return

        if pcd is None:
            self.get_logger().warn('Empty PointCloud atrfter ansformation.')
            return

        self.get_logger().info(f'PointCloud has {len(pcd.points)} points after transformation.')

        # Filter points within the cuboid
        filtered_pcd = self.filter_points_in_cuboid(pcd)

        if filtered_pcd is None or len(filtered_pcd.points) == 0:
            self.get_logger().warn('No points found within the specified cuboid.')
            return

        self.get_logger().info(f'Filtered PointCloud has {len(filtered_pcd.points)} points inside the cuboid.')

        # Create a new header with frame_id set to 'camera_base_parallel_floor'
        new_header = Header()
        new_header.stamp = self.get_clock().now().to_msg()
        new_header.frame_id = 'camera_base_parallel_floor' 

        # Convert filtered Open3D point cloud back to ROS PointCloud2
        filtered_msg = o3d_to_pointcloud2(filtered_pcd, new_header)
        if filtered_msg is not None:
            self.filtered_publisher.publish(filtered_msg)
            self.get_logger().info('Published filtered PointCloud2 message.')

        # Optional: Visualize the filtered point cloud with cuboid boundaries
        if not self.visualize:
            return
        
        with self.visualization_lock:
            if self.visualization_thread is None or not self.visualization_thread.is_alive():
                self.visualization_thread = threading.Thread(
                    target=visualize_point_cloud,
                    args=(
                        filtered_pcd,
                        (self.cuboid_center_x, self.cuboid_center_y, self.cuboid_center_z),
                        (self.cuboid_length_x, self.cuboid_length_y, self.cuboid_length_z)
                    )
                )
                self.visualization_thread.start()

    def transform_pointcloud(self, pc2_msg, transform_stamped):
        """
        Transform the incoming PointCloud2 message to the target frame using the provided transform.

        Args:
            pc2_msg (PointCloud2): The incoming point cloud message.
            transform_stamped (TransformStamped): The transformation to apply.

        Returns:
            open3d.geometry.PointCloud: The transformed point cloud.
        """
        # Convert ROS PointCloud2 to Open3D
        pcd = pointcloud2_to_o3d(pc2_msg)
        if pcd is None:
            return None

        # Convert transform to 4x4 matrix
        transformation_matrix = self.transform_stamped_to_matrix(transform_stamped)

        # Apply transformation
        pcd.transform(transformation_matrix)

        return pcd

    def transform_stamped_to_matrix(self, transform_stamped):
        """
        Convert a TransformStamped message to a 4x4 transformation matrix.

        Args:
            transform_stamped (TransformStamped): The transformation message.

        Returns:
            numpy.ndarray: The 4x4 transformation matrix.
        """
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

    def filter_points_in_cuboid(self, pcd):
        """
        Filter points that lie within the specified cuboid.

        Args:
            pcd (open3d.geometry.PointCloud): The input point cloud.

        Returns:
            open3d.geometry.PointCloud: The filtered point cloud within the cuboid.
        """
        # Convert point cloud to NumPy array
        points = np.asarray(pcd.points)

        # Define cuboid boundaries
        x_min = self.cuboid_center_x - self.cuboid_length_x / 2
        x_max = self.cuboid_center_x + self.cuboid_length_x / 2
        y_min = self.cuboid_center_y - self.cuboid_length_y / 2
        y_max = self.cuboid_center_y + self.cuboid_length_y / 2
        z_min = self.cuboid_center_z - self.cuboid_length_z / 2
        z_max = self.cuboid_center_z + self.cuboid_length_z / 2

        self.get_logger().info(f"Cuboid Bounds: X[{x_min}, {x_max}], Y[{y_min}, {y_max}], Z[{z_min}, {z_max}]")

        # Create a boolean mask for points inside the cuboid
        mask = (
            (points[:, 0] >= x_min) & (points[:, 0] <= x_max) &
            (points[:, 1] >= y_min) & (points[:, 1] <= y_max) &
            (points[:, 2] >= z_min) & (points[:, 2] <= z_max)
        )

        # Apply the mask to extract the filtered points
        filtered_points = points[mask]

        if filtered_points.size == 0:
            return None

        # Create a new Open3D point cloud with the filtered points
        filtered_pcd = o3d.geometry.PointCloud()
        filtered_pcd.points = o3d.utility.Vector3dVector(filtered_points)

        return filtered_pcd

def main(args=None):
    rclpy.init(args=args)
    node = SegmentationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Segmentation Node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
