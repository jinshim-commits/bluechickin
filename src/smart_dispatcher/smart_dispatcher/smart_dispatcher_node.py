import json
import random

import rclpy
from rclpy.node import Node

# [확인] String 메시지 타입이 꼭 필요합니다 (이미 있다면 패스)
from std_msgs.msg import String, Float32, Bool
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue


DEPARTMENT_COORDINATES = {
    "진단검사의학과": {"x": 0.48070189356803894, "y": 0.2762919068336487, "w": 1.0},
    "영상의학과":    {"x": 6.578537940979004,  "y": 2.621462106704712,  "w": 1.0},
    "내과":          {"x": 7.445363998413086,  "y": 0.5102964639663696, "w": 1.0},
    "정형외과":      {"x": 0.753912627696991,  "y": -2.640972375869751, "w": 1.0},
    "안내데스크":    {"x": 2.836460590362549,  "y": 1.1752597093582153, "w": 1.0},
}
INFO_DESK_NAME = "안내데스크"


class SmartDispatcher(Node):
    def __init__(self):
        super().__init__('smart_dispatcher')

        # ---- 상태 ----
        self.remaining_depts = []
        self.waiting_counts = {}
        self.wait_min = 0
        self.wait_max = 20

        self.current_goal_name = None
        self.current_goal_pose = None
        self.waiting_next = False
        self.is_paused = False
        self.is_emergency = False

        # ---- home 저장 ----
        self.home_pose = None
        self.home_saved = False
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.cb_amcl_pose, 10)

        # ---- Nav2 ----
        self.navigator = BasicNavigator()
        # self.navigator.waitUntilNav2Active() # 시뮬레이션 상황에 따라 주석 해제

        # ---- 속도 ----
        self.current_speed = self._get_initial_speed_from_velocity_smoother()
        self.min_speed = 0.10
        self.max_speed = 0.40

        # ========================================================
        # [추가됨 1] 도착 알림용 확성기(Publisher) 설치
        # ========================================================
        # UI 패키지(doctor_ui_trigger)가 이 토픽을 듣고 반응합니다.
        self.pub_arrival_status = self.create_publisher(String, '/hospital/arrival_status', 10)

        # ---- Sub (입력) ----
        self.create_subscription(String,  '/hospital/patient_data',   self.cb_patient_data, 10)
        self.create_subscription(Bool,    '/hospital/next_waypoint',  self.cb_next_waypoint, 10)
        self.create_subscription(Float32, '/nav_speed_delta',         self.cb_speed, 10)
        self.create_subscription(Bool,    '/nav_pause',               self.cb_pause, 10)
        self.create_subscription(Bool,    '/nav_emergency_home',      self.cb_emergency_home, 10)

        self.get_logger().info("IDLE: QR 대기 중 (UI Trigger Ready)")

        # ---- 주기 타이머 ----
        self.create_timer(0.1, self.loop)

    # =============== 콜백들 (기존과 동일) ===============
    def cb_amcl_pose(self, msg: PoseWithCovarianceStamped):
        if self.home_saved: return
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose = msg.pose.pose
        self.home_pose = pose
        self.home_saved = True
        self.get_logger().info("[dispatcher] Home pose saved")

    def cb_patient_data(self, msg: String):
        if self.is_emergency: return
        try:
            data = json.loads(msg.data)
            depts = data.get("departments", [])
        except Exception: return

        self.remaining_depts = [d for d in depts if (d in DEPARTMENT_COORDINATES) and (d != INFO_DESK_NAME)]
        if not self.remaining_depts: return

        self.get_logger().info("READY: 첫 목적지 출발")
        self.waiting_next = False
        self.is_paused = False
        self.is_emergency = False
        self._start_next_goal()

    def cb_next_waypoint(self, msg: Bool):
        if not msg.data or self.is_emergency: return
        if self.waiting_next:
            self.waiting_next = False
            self.get_logger().info("MOVING: 다음 목적지 출발")
            self._start_next_goal()

    def cb_speed(self, msg: Float32):
        self.current_speed = float(self.current_speed) + float(msg.data)
        self.current_speed = max(self.min_speed, min(self.current_speed, self.max_speed))
        self._apply_speed(self.current_speed)

    def cb_pause(self, msg: Bool):
        if msg.data:
            self.is_paused = True
            self.navigator.cancelTask()
            return
        self.is_paused = False
        if self.is_emergency: return
        if self.waiting_next: return
        if self.current_goal_pose:
            self.navigator.goToPose(self.current_goal_pose)

    def cb_emergency_home(self, msg: Bool):
        if not msg.data: return
        self.is_emergency = True
        self.is_paused = False
        self.waiting_next = False
        self.remaining_depts = []
        self.waiting_counts = {}
        self.current_goal_name = None
        self.current_goal_pose = None
        self.navigator.cancelTask()
        if self.home_pose is None:
            # Home pose fallback
            self.home_pose = PoseStamped()
            self.home_pose.header.frame_id = "map"
            self.home_pose.pose.position.x = 0.0
            self.home_pose.pose.position.y = 0.0
            self.home_pose.pose.orientation.w = 1.0
        self.get_logger().info("EMERGENCY: HOME 복귀")
        self.navigator.goToPose(self.home_pose)

    # =============== 메인 루프 (여기가 중요!) ===============
    def loop(self):
        # 1. Emergency 복귀 중일 때
        if self.is_emergency:
            if self.navigator.isTaskComplete():
                self.is_emergency = False
                self.get_logger().info("EMERGENCY DONE: HOME 도착")
            return

        if self.is_paused or self.waiting_next:
            return

        # 2. 일반 주행 중일 때
        if self.current_goal_pose is not None:
            if self.navigator.isTaskComplete():
                res = self.navigator.getResult()
                
                if res == TaskResult.SUCCEEDED:
                    self.get_logger().info(f"ARRIVED: {self.current_goal_name}")
                    
                    # ========================================================
                    # [추가됨 2] 도착 성공 시 방송 내보내기 📢
                    # ========================================================
                    msg = String()
                    msg.data = self.current_goal_name  # 예: "내과"
                    self.pub_arrival_status.publish(msg)
                    # ========================================================

                else:
                    self.get_logger().info(f"FAILED: {self.current_goal_name}")

                # 도착했거나 실패했으니, 다음 명령 전까지 대기
                self.waiting_next = True

    # =============== 유틸 (기존과 동일) ===============
    def _refresh_waiting_counts(self):
        self.waiting_counts = {d: random.randint(self.wait_min, self.wait_max) for d in self.remaining_depts}

    def _start_next_goal(self):
        if not self.remaining_depts:
            self.current_goal_name = None
            self.current_goal_pose = None
            self.get_logger().info("DONE: 모든 waypoint 완료")
            return

        self._refresh_waiting_counts()
        min_wait = min(self.waiting_counts.values())
        candidates = [d for d, w in self.waiting_counts.items() if w == min_wait]
        name = random.choice(candidates)
        self.remaining_depts.remove(name)

        info = DEPARTMENT_COORDINATES[name]
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(info["x"])
        pose.pose.position.y = float(info["y"])
        pose.pose.orientation.w = float(info.get("w", 1.0))

        self.current_goal_name = name
        self.current_goal_pose = pose
        self.get_logger().info(f"MOVING: {name}")
        self.navigator.goToPose(pose)

    def _get_initial_speed_from_velocity_smoother(self) -> float:
        # (기존 코드와 동일, 생략하거나 그대로 두세요)
        return 0.25

    def _apply_speed(self, speed: float):
        self._set_remote_param('/controller_server', 'FollowPath.max_vel_x', speed)
        self._set_remote_param('/velocity_smoother', 'max_velocity', [speed, 0.0, 1.0])

    def _set_remote_param(self, node_name: str, param_name: str, value):
        client = self.create_client(SetParameters, f'{node_name}/set_parameters')
        if not client.service_is_ready(): return
        p = Parameter()
        p.name = param_name
        if isinstance(value, list):
            p.value = ParameterValue(type=ParameterValue.TYPE_DOUBLE_ARRAY, double_array_value=[float(x) for x in value])
        else:
            p.value = ParameterValue(type=ParameterValue.TYPE_DOUBLE, double_value=float(value))
        req = SetParameters.Request()
        req.parameters = [p]
        client.call_async(req)


def main():
    rclpy.init()
    node = SmartDispatcher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
