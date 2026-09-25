#!/usr/bin/env python3

import math
import os
import time
from typing import Dict, List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

try:
    from gazebo_msgs.msg import EntityState
    from gazebo_msgs.srv import SetEntityState
    HAS_ENTITY_STATE = True
except Exception:
    HAS_ENTITY_STATE = False

try:
    from gazebo_msgs.msg import ModelState
    from gazebo_msgs.srv import SetModelState
    HAS_MODEL_STATE = True
except Exception:
    HAS_MODEL_STATE = False


def yaw_to_quaternion(yaw: float):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def shortest_angle_delta(start: float, end: float) -> float:
    return math.atan2(math.sin(end - start), math.cos(end - start))


def interpolate_yaw(start: float, end: float, fraction: float) -> float:
    fraction = max(0.0, min(1.0, fraction))
    return start + shortest_angle_delta(start, end) * fraction


class DynamicObstacleController(Node):
    """
    Safer dynamic obstacle controller.

    Changes from the previous version:
      - Animals are distributed around the farm, not all crossing in front of the UGV.
      - Motion is slow and includes pauses.
      - The person crosses occasionally, not continuously.
      - Z is kept stable to reduce jumping/flying behavior.
      - Uses Gazebo set_entity_state or set_model_state services.
    """

    def __init__(self):
        super().__init__("dynamic_obstacle_controller")
        self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])

        self.marker_pub = self.create_publisher(MarkerArray, "/dynamic_obstacles/markers", 10)
        scenario_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            String,
            "/benchmark/scenario",
            self.scenario_callback,
            scenario_qos,
        )

        self.scenario = os.environ.get("UAV_UGV_SCENARIO", "S1").upper()
        self.static_parking_poses: Dict[str, Tuple[float, float, float]] = {
            "cow_1": (-18.0, -18.0, 0.0),
            "sheep_1": (-19.5, -18.0, 0.0),
            "dog_1": (-18.0, -19.5, 0.0),
            "person_1": (-19.5, -19.5, 0.0),
        }

        # Waypoints are chosen to avoid trapping the robot at the start area.
        # Format:
        #   points: patrol points
        #   speed: approximate m/s
        #   pause: seconds to wait at each waypoint
        #   z: fixed model z
        self.paths: Dict[str, Dict] = {
            "cow_1": {
                "points": [(-4.0, 5.2), (-1.0, 5.2), (-1.0, 6.6), (-4.0, 6.6)],
                "speed": 0.18,
                "pause": 3.0,
                "z": 0.05,
            },
            "sheep_1": {
                "points": [(8.0, -2.8), (10.5, -2.8), (10.5, -1.2), (8.0, -1.2)],
                "speed": 0.20,
                "pause": 2.5,
                "z": 0.05,
            },
            "dog_1": {
                "points": [(-7.2, -2.5), (-4.8, -2.5), (-4.8, -4.0), (-7.2, -4.0)],
                "speed": 0.24,
                "pause": 2.0,
                "z": 0.05,
            },
            # Person crosses only occasionally near the working area.
            # It waits at both sides, giving Nav2 time to pass after replanning.
            "person_1": {
                "points": [(5.5, 3.8), (5.5, -1.8)],
                "speed": 0.16,
                "pause": 7.0,
                "z": 0.05,
            },
        }

        self.start_time = None
        # S1/S2 have no dynamic obstacles. Park the models once instead of
        # hammering Gazebo with 16 unnecessary set-state service calls/second.
        # S3 remains continuously updated at the existing 4 Hz cadence.
        self.static_parking_applied = False
        self.max_corner_angular_velocity = 0.65
        self.last_service_warning_time = 0.0
        self.last_move_log_time = 0.0

        self.entity_clients = []
        self.model_clients = []

        if HAS_ENTITY_STATE:
            for srv_name in ["/gazebo/set_entity_state", "/set_entity_state"]:
                self.entity_clients.append((srv_name, self.create_client(SetEntityState, srv_name)))

        if HAS_MODEL_STATE:
            for srv_name in ["/gazebo/set_model_state", "/set_model_state"]:
                self.model_clients.append((srv_name, self.create_client(SetModelState, srv_name)))

        self.timer = self.create_timer(0.25, self.update_obstacles)

        self.get_logger().info("Dynamic obstacle controller started")
        self.get_logger().info(f"Benchmark scenario mode active: {self.scenario}")
        self.get_logger().info("Moving requested models: cow_1, sheep_1, dog_1, person_1")

    def scenario_callback(self, msg: String):
        scenario = msg.data.strip().upper()
        if scenario not in {"S1", "S2", "S3"}:
            self.get_logger().warn(
                f"Unknown benchmark scenario '{msg.data}', keeping {self.scenario}"
            )
            return
        if scenario == self.scenario:
            return
        self.scenario = scenario
        self.start_time = None
        self.static_parking_applied = False
        self.get_logger().info(f"Benchmark scenario mode active: {self.scenario}")

    def build_segments(self, points: List[Tuple[float, float]]):
        n = len(points)
        if n == 2:
            return [(points[0], points[1]), (points[1], points[0])]
        return [(points[i], points[(i + 1) % n]) for i in range(n)]

    def segment_yaw(self, p0: Tuple[float, float], p1: Tuple[float, float]) -> float:
        return math.atan2(p1[1] - p0[1], p1[0] - p0[0])

    def corner_pause_duration(
        self,
        incoming_yaw: float,
        outgoing_yaw: float,
        configured_pause: float,
    ) -> Tuple[float, float]:
        turn = abs(shortest_angle_delta(incoming_yaw, outgoing_yaw))
        rotation_duration = turn / max(self.max_corner_angular_velocity, 0.001)
        return max(configured_pause, rotation_duration), rotation_duration

    def path_total_time(self, points: List[Tuple[float, float]], speed: float, pause: float):
        total = 0.0
        segments = self.build_segments(points)

        for i, (p0, p1) in enumerate(segments):
            prev_p0, prev_p1 = segments[i - 1]
            pause_duration, _ = self.corner_pause_duration(
                self.segment_yaw(prev_p0, prev_p1),
                self.segment_yaw(p0, p1),
                pause,
            )
            dist = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
            total += pause_duration + dist / max(speed, 0.01)

        return total, segments

    def interpolate_path(self, points: List[Tuple[float, float]], speed: float, pause: float, t: float):
        total_time, segments = self.path_total_time(points, speed, pause)
        phase_time = t % total_time

        for i, (p0, p1) in enumerate(segments):
            prev_p0, prev_p1 = segments[i - 1]
            incoming_yaw = self.segment_yaw(prev_p0, prev_p1)
            outgoing_yaw = self.segment_yaw(p0, p1)
            pause_duration, rotation_duration = self.corner_pause_duration(
                incoming_yaw,
                outgoing_yaw,
                pause,
            )
            dist = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
            travel_time = dist / max(speed, 0.01)

            # Pause at p0 and rotate continuously before the next straight segment.
            if phase_time <= pause_duration:
                if rotation_duration > 0.001:
                    yaw = interpolate_yaw(
                        incoming_yaw,
                        outgoing_yaw,
                        phase_time / rotation_duration,
                    )
                else:
                    yaw = outgoing_yaw
                return p0[0], p0[1], yaw

            phase_time -= pause_duration

            # Travel p0 -> p1.
            if phase_time <= travel_time:
                local = phase_time / max(travel_time, 0.001)
                x = p0[0] + (p1[0] - p0[0]) * local
                y = p0[1] + (p1[1] - p0[1]) * local
                yaw = outgoing_yaw
                return x, y, yaw

            phase_time -= travel_time

        p = points[-1]
        return p[0], p[1], 0.0

    def make_pose(self, x: float, y: float, z: float, yaw: float) -> Pose:
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        qz, qw = yaw_to_quaternion(yaw)
        pose.orientation.z = qz
        pose.orientation.w = qw
        return pose

    def get_ready_entity_client(self):
        for name, client in self.entity_clients:
            if client.service_is_ready():
                return name, client
        return None, None

    def get_ready_model_client(self):
        for name, client in self.model_clients:
            if client.service_is_ready():
                return name, client
        return None, None

    def send_state(self, name: str, x: float, y: float, z: float, yaw: float) -> bool:
        pose = self.make_pose(x, y, z, yaw)

        entity_srv_name, entity_client = self.get_ready_entity_client()
        if entity_client is not None:
            req = SetEntityState.Request()
            req.state = EntityState()
            req.state.name = name
            req.state.pose = pose
            req.state.twist = Twist()
            req.state.reference_frame = "world"
            entity_client.call_async(req)
            return True

        model_srv_name, model_client = self.get_ready_model_client()
        if model_client is not None:
            req = SetModelState.Request()
            req.model_state = ModelState()
            req.model_state.model_name = name
            req.model_state.pose = pose
            req.model_state.twist = Twist()
            req.model_state.reference_frame = "world"
            model_client.call_async(req)
            return True

        now = time.time()
        if now - self.last_service_warning_time > 3.0:
            self.last_service_warning_time = now
            self.get_logger().warn(
                "No Gazebo state service is ready. Run: ros2 service list | grep -E 'set_.*state|gazebo'"
            )
        return False

    def publish_markers(self, poses: Dict[str, Tuple[float, float, float]]):
        arr = MarkerArray()
        for i, (name, (x, y, yaw)) in enumerate(poses.items()):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "dynamic_obstacles"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.45
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.75
            marker.scale.y = 0.75
            marker.scale.z = 0.9
            marker.color.r = 1.0
            marker.color.g = 0.45
            marker.color.b = 0.0
            marker.color.a = 0.65
            arr.markers.append(marker)
        self.marker_pub.publish(arr)

    def update_obstacles(self):
        poses = {}
        moved_any = False

        if self.scenario == "S3":
            now_sim = self.get_clock().now().nanoseconds / 1e9
            if self.start_time is None:
                self.start_time = now_sim
            t = now_sim - self.start_time
            for name, cfg in self.paths.items():
                x, y, yaw = self.interpolate_path(cfg["points"], cfg["speed"], cfg["pause"], t)
                z = cfg.get("z", 0.05)
                ok = self.send_state(name, x, y, z, yaw)
                moved_any = moved_any or ok
                poses[name] = (x, y, yaw)
        else:
            poses.update(self.static_parking_poses)
            if not self.static_parking_applied:
                parking_submitted = True
                for name, (x, y, yaw) in self.static_parking_poses.items():
                    ok = self.send_state(name, x, y, 0.05, yaw)
                    parking_submitted = parking_submitted and ok
                    moved_any = moved_any or ok
                if parking_submitted:
                    self.static_parking_applied = True
                    self.get_logger().info(
                        f"Static obstacle parking submitted once for {self.scenario}; "
                        "continuous Gazebo set-state traffic disabled."
                    )

        self.publish_markers(poses)

        now = time.time()
        if moved_any and now - self.last_move_log_time > 5.0:
            self.last_move_log_time = now
            if self.scenario == "S3":
                self.get_logger().info("Publishing dynamic patrol obstacle states to Gazebo")
            else:
                self.get_logger().info(
                    f"Publishing static off-map obstacle parking states for {self.scenario}"
                )


def main():
    rclpy.init()
    node = DynamicObstacleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
