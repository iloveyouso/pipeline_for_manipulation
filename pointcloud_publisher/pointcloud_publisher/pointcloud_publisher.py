#!/usr/bin/env python3
import socket
import struct
import io
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

def pointcloud2_to_xyz_array(msg):
    """
    간단한 변환 함수.
    가정: PointCloud2 메시지는 각 점에 대해 x,y,z가 float32 3개(총 12바이트)로 저장되어 있음.
    """
    num_points = msg.width * msg.height
    cloud = np.frombuffer(msg.data, dtype=np.float32)
    if cloud.size != num_points * 3:
        print("Warning: 예상 데이터 크기와 다름. 처음 num_points*3 요소만 사용합니다.")
        cloud = cloud[:num_points * 3]
    xyz = cloud.reshape(num_points, 3)
    return xyz

class PointCloudSocketBridge(Node):
    def __init__(self, host, port):
        super().__init__('pointcloud_socket_bridge')
        self.host = host
        self.port = port
        # /filtered_pc 토픽 구독
        self.subscription = self.create_subscription(
            PointCloud2,
            '/filtered_points',
            self.pc_callback,
            10
        )
        self.get_logger().info("Subscribed to /filtered_points")

    def pc_callback(self, msg: PointCloud2):
        # PointCloud2 메시지를 numpy 배열로 변환
        xyz = pointcloud2_to_xyz_array(msg)
        # np.save를 사용해 메모리 내 BytesIO 버퍼에 저장
        buffer = io.BytesIO()
        np.save(buffer, xyz)
        data = buffer.getvalue()
        data_length = len(data)
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect((self.host, self.port))
                # 먼저 4바이트 정수로 데이터 길이를 전송한 후, npy 데이터를 전송
                s.sendall(struct.pack('I', data_length))
                s.sendall(data)
                self.get_logger().info(f"Sent numpy array of shape {xyz.shape} with size {data_length} bytes")
        except Exception as e:
            self.get_logger().error(f"Socket error: {e}")

def main(args=None):
    rclpy.init(args=args)
    host = 'localhost'
    port = 65433  # 수신측과 맞춰줍니다.
    node = PointCloudSocketBridge(host, port)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
