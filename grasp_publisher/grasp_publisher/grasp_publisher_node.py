#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
import numpy as np
import tf_transformations
import socket
import json
import threading

class GraspPublisher(Node):
    def __init__(self, host='localhost', port=65432):
        super().__init__('grasp_publisher')

        # PoseStamped 메시지만 퍼블리시하도록 퍼블리셔 생성
        self.pose_pub = self.create_publisher(PoseStamped, 'grasp_poses', 10)

        # Socket 서버 파라미터
        self.host = host
        self.port = port

        # 소켓 서버를 별도의 스레드로 실행
        server_thread = threading.Thread(target=self.start_socket_server, daemon=True)
        server_thread.start()

        self.get_logger().info(f"Grasp Publisher Node initialized. Listening on {self.host}:{self.port}")

    def start_socket_server(self):
        """
        소켓 서버를 시작하여 grasp 데이터를 수신합니다.
        """
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((self.host, self.port))
            s.listen()
            self.get_logger().info(f"Socket server listening on {self.host}:{self.port}")
            while True:
                conn, addr = s.accept()
                with conn:
                    self.get_logger().info(f"Connected by {addr}")
                    data_buffer = ""
                    while True:
                        data = conn.recv(4096)
                        if not data:
                            break
                        data_buffer += data.decode('utf-8')
                        try:
                            while True:
                                json_obj, idx = self.extract_json(data_buffer)
                                if json_obj is None:
                                    break
                                self.process_grasp_data(json_obj)
                                data_buffer = data_buffer[idx:]
                        except json.JSONDecodeError as e:
                            self.get_logger().error(f"JSON decode error: {e}")
                            break

    def extract_json(self, data):
        """
        문자열에서 완전한 JSON 객체를 추출합니다.
        JSON 객체와 파싱된 인덱스를 반환합니다.
        """
        decoder = json.JSONDecoder()
        try:
            obj, idx = decoder.raw_decode(data)
            return obj, idx
        except json.JSONDecodeError:
            return None, None

    def process_grasp_data(self, data):
        """
        수신한 grasp 데이터를 처리하여 PoseStamped 메시지로 퍼블리시합니다.
        """
        try:
            generated_grasps = data['generated_grasps']
            # grasp matrix들을 NumPy 배열로 변환
            grasp_matrices = [np.array(grasp) for grasp in generated_grasps]

            self.publish_grasps(grasp_matrices)

            self.get_logger().info(f"Received and published {len(grasp_matrices)} grasp(s).")
        except KeyError as e:
            self.get_logger().error(f"Missing key in data: {e}")
        except Exception as e:
            self.get_logger().error(f"Error processing grasp data: {e}")

    def publish_grasps(self, grasp_matrices):
        """
        각 grasp matrix를 PoseStamped 메시지로 변환하여 퍼블리시합니다.
        """
        for mat in grasp_matrices:
            pose = self.matrix_to_pose(mat)
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            # 필요에 따라 프레임 아이디 수정 (예: "camera_base_parallel_floor")
            pose_stamped.header.frame_id = "camera_base_parallel_floor"
            pose_stamped.pose = pose
            self.pose_pub.publish(pose_stamped)

    def matrix_to_pose(self, mat: np.ndarray) -> Pose:
        """
        4x4 변환 행렬을 geometry_msgs/Pose 메시지로 변환합니다.
        """
        # 평행 이동 추출
        tx = mat[0, 3]
        ty = mat[1, 3]
        tz = mat[2, 3]

        # 회전행렬에서 quaternion 추출
        rotation_matrix = mat[0:3, 0:3]
        full_4x4 = np.eye(4)
        full_4x4[0:3, 0:3] = rotation_matrix
        quat = tf_transformations.quaternion_from_matrix(full_4x4)

        pose_msg = Pose()
        pose_msg.position.x = float(tx)
        pose_msg.position.y = float(ty)
        pose_msg.position.z = float(tz)
        pose_msg.orientation.x = float(quat[0])
        pose_msg.orientation.y = float(quat[1])
        pose_msg.orientation.z = float(quat[2])
        pose_msg.orientation.w = float(quat[3])
        return pose_msg

def main(args=None):
    rclpy.init(args=args)
    node = GraspPublisher(host='0.0.0.0', port=65432)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
