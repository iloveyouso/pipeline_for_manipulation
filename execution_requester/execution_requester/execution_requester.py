import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped

# MoveIt 관련 메시지 및 액션
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, PlanningOptions, Constraints, PositionConstraint, OrientationConstraint, RobotState
from shape_msgs.msg import SolidPrimitive

class MoveItGraspClient(Node):
    def __init__(self):
        super().__init__('moveit_grasp_client')
        self.get_logger().info("MoveItGraspClient node started")
        
        # MoveGroup 액션 클라이언트 생성 (액션 서버 이름은 보통 'move_group')
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        
        # /grasp_poses 토픽 구독 (메시지 타입: PoseStamped)
        self._subscription = self.create_subscription(
            PoseStamped,
            '/grasp_poses',
            self.grasp_pose_callback,
            10
        )
        
        # 설정 값 (사용중인 MoveIt 설정에 맞게 수정)
        self.planning_group = "arm"
        self.target_link = "gripper_base_link"
        print("MoveItGraspClient successfully initialized")

    def grasp_pose_callback(self, pose_msg: PoseStamped):
        self.get_logger().info("Received grasp pose message")
        # MotionPlanRequest 생성
        request = MotionPlanRequest()
        request.group_name = self.planning_group
        request.num_planning_attempts = 10
        request.allowed_planning_time = 5.0
        
        # start_state를 비워두면 planning 서버가 현재 상태를 사용합니다.
        request.start_state = RobotState()

        # 목표 pose를 Constraints로 변환
        constraint = Constraints()
        constraint.name = "goal_constraint"

        # -- 위치 제약(Position Constraint) --
        pos_constraint = PositionConstraint()
        pos_constraint.header = pose_msg.header
        pos_constraint.link_name = self.target_link
        pos_constraint.target_point_offset.x = 0.0
        pos_constraint.target_point_offset.y = 0.0
        pos_constraint.target_point_offset.z = 0.0

        # 간단하게 구형 영역(반경 1cm)을 제약 영역으로 사용
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01]  # 반경 1cm

        # 제약 영역 설정
        pos_constraint.constraint_region.primitives.append(sphere)
        pos_constraint.constraint_region.primitive_poses.append(pose_msg.pose)
        pos_constraint.weight = 1.0
        constraint.position_constraints.append(pos_constraint)

        # -- 자세 제약(Orientation Constraint) --
        ori_constraint = OrientationConstraint()
        ori_constraint.header = pose_msg.header
        ori_constraint.link_name = self.target_link
        ori_constraint.orientation = pose_msg.pose.orientation
        # 각 축에 대해 허용 오차 (라디안 단위, 필요에 따라 조절)
        ori_constraint.absolute_x_axis_tolerance = 0.1
        ori_constraint.absolute_y_axis_tolerance = 0.1
        ori_constraint.absolute_z_axis_tolerance = 0.1
        ori_constraint.weight = 1.0
        constraint.orientation_constraints.append(ori_constraint)

        # MotionPlanRequest에 목표 제약 추가
        request.goal_constraints.append(constraint)

        # PlanningOptions (추가 옵션 설정, 예: diff 모드 활성화)
        options = PlanningOptions()
        options.planning_scene_diff.is_diff = True

        # MoveGroup 액션 goal 메시지 구성
        goal_msg = MoveGroup.Goal()
        goal_msg.request = request
        goal_msg.planning_options = options

        self.get_logger().info("Sending motion plan request to MoveGroup action server")
        self.send_goal(goal_msg)

    def send_goal(self, goal_msg: MoveGroup.Goal):
        # 액션 서버가 준비될 때까지 대기
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("MoveGroup action server not available!")
            return
        
        # 액션 목표 전송 (피드백 콜백도 등록 가능)
        send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by MoveGroup action server")
            return
        self.get_logger().info("Goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        # 피드백 메시지 처리 (원한다면 진행 상황을 출력)
        feedback = feedback_msg.feedback
        self.get_logger().info("Feedback received: %s" % str(feedback))

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info("Motion planning result received")
        # result.error_code.val 값이 1이면 성공(SUCCESS)로 간주 (MoveItErrorCodes 참고)
        if result.error_code.val == 1:
            self.get_logger().info("Motion executed successfully")
        else:
            self.get_logger().error("Motion execution failed with error code: %d" % result.error_code.val)

def main(args=None):
    rclpy.init(args=args)
    node = MoveItGraspClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
