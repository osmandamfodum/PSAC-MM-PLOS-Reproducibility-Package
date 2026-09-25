#!/usr/bin/env python3

import os
import time
import math
import hashlib
import json
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image
from std_msgs.msg import String, UInt32
from visualization_msgs.msg import Marker, MarkerArray
from ament_index_python.packages import get_package_share_directory

try:
    from gazebo_msgs.msg import ModelStates
    HAS_GAZEBO_MSGS = True
except Exception:
    HAS_GAZEBO_MSGS = False


class UAVFarmScanner(Node):
    """
    Simulated UAV aerial survey node.

    Outputs:
      /uav/tree_target            PoseStamped       every unique treatment target
      /uav/scan_complete          UInt32            total unique treatment targets
      /uav/aerial_image           sensor_msgs/Image  synthetic top-down aerial image
      /uav/aerial_obstacle_map    OccupancyGrid      drone-derived obstacle map
      /uav/farm_markers           MarkerArray        RViz markers for targets, obstacles, and UAV pose
      /uav/status                 String             survey status

    Behavior:
      - Simulates UAV takeoff and farm survey.
      - Reads farm.world and optionally live Gazebo model states.
      - Detects crops/trees needing treatment.
      - Detects obstacles: rocks, fence, animals, humans/persons.
      - Publishes a top-down "aerial image" and obstacle map.
      - Publishes treatment targets in map frame.
    """

    def __init__(self):
        super().__init__("uav_farm_scanner")
        self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])

        target_qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        scan_complete_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.target_pub = self.create_publisher(
            PoseStamped,
            "/uav/tree_target",
            target_qos,
        )
        self.scan_complete_pub = self.create_publisher(
            UInt32,
            "/uav/scan_complete",
            scan_complete_qos,
        )
        self.image_pub = self.create_publisher(Image, "/uav/aerial_image", 10)
        self.map_pub = self.create_publisher(OccupancyGrid, "/uav/aerial_obstacle_map", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/uav/farm_markers", 10)
        self.status_pub = self.create_publisher(String, "/uav/status", 10)
        self.survey_report_pub = self.create_publisher(String, "/uav/survey_report", 10)
        self.uav_pose_pub = self.create_publisher(PoseStamped, "/uav/scout_pose", 10)

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
        self.scenario_target_names = {
            # Open/static farm: omit the difficult crop-row access target.
            "S1": {"crop_row1_6", "tree_1", "tree_2"},
            # Constrained crop-row benchmark: includes the difficult (-6, 4.5) target.
            "S2": {"crop_row1_6", "crop_row2_2", "tree_1"},
            # Dynamic-obstacle scenario uses the same target set as S2.
            "S3": {"crop_row1_6", "crop_row2_2", "tree_1"},
        }

        self.world_file = Path(get_package_share_directory("my_robot_description")) / "worlds" / "farm.world"

        # If your Gazebo and map frames are shifted, set these offsets.
        self.map_offset_x = 0.0
        self.map_offset_y = 0.0

        # UAV survey region in map/world coordinates.
        self.min_x = -12.0
        self.max_x = 22.0
        self.min_y = -8.0
        self.max_y = 12.0

        # Aerial occupancy grid resolution.
        self.grid_resolution = 0.10
        self.grid_width = int((self.max_x - self.min_x) / self.grid_resolution)
        self.grid_height = int((self.max_y - self.min_y) / self.grid_resolution)

        # Synthetic aerial image size.
        self.image_width = 700
        self.image_height = 420

        # UAV simulated flight.
        self.uav_altitude = 5.0
        self.survey_started_at = None
        self.takeoff_seconds = 3.0
        self.survey_period_sec = 4.0

        self.allowed_target_prefixes = ["tree_", "crop_row"]
        self.obstacle_prefixes = [
            "rock",
            "fence",
            "cow",
            "sheep",
            "dog",
            "goat",
            "horse",
            "animal",
            "person",
            "human",
            "worker",
        ]
        self.ignored_prefixes = ["ground", "sun", "uav_scout_drone"]

        # Approximate aerial obstacle radius by model type.
        self.default_obstacle_radius = 0.55
        self.obstacle_radius_by_prefix = {
            "rock": 0.85,
            "fence": 0.80,
            "cow": 1.05,
            "sheep": 0.70,
            "dog": 0.55,
            "person": 0.65,
            "human": 0.65,
            "worker": 0.65,
            "tree_": 0.50,
            "crop_row": 0.35,
        }

        # A treatment target is published once per scanner run.
        self.sent_targets: Dict[str, Dict] = {}
        self.scan_complete_sent = False
        self.last_treatment_signature: Tuple[str, ...] = ()
        self.stable_treatment_scan_count = 0
        self.required_stable_treatment_scans = 2

        self.live_model_poses: Dict[str, Tuple[float, float, float]] = {}

        if HAS_GAZEBO_MSGS:
            self.create_subscription(ModelStates, "/gazebo/model_states", self.model_states_callback, 10)
            self.get_logger().info("Subscribed to /gazebo/model_states for live obstacle positions")
        else:
            self.get_logger().warn("gazebo_msgs not available; using farm.world static poses only")

        self.timer = self.create_timer(self.survey_period_sec, self.scan_farm)

        self.publish_status("UAV scanner initialized")
        self.get_logger().info("UAV aerial farm scanner started")
        self.get_logger().info(f"Scenario target filter active: {self.scenario}")
        self.get_logger().info(f"Watching world file: {self.world_file}")
        self.get_logger().info("Publishing aerial image, obstacle map, markers, and treatment targets")

    # ------------------------------------------------------------------
    # Gazebo/live pose integration
    # ------------------------------------------------------------------

    def model_states_callback(self, msg):
        for name, pose in zip(msg.name, msg.pose):
            self.live_model_poses[name] = (
                pose.position.x + self.map_offset_x,
                pose.position.y + self.map_offset_y,
                pose.position.z,
            )

    def scenario_callback(self, msg: String):
        scenario = msg.data.strip().upper()
        if scenario not in self.scenario_target_names:
            self.get_logger().warn(
                f"Unknown benchmark scenario '{msg.data}', keeping {self.scenario}"
            )
            return
        if scenario == self.scenario:
            return
        if self.sent_targets:
            self.get_logger().warn(
                "Scenario changed after target publication started; "
                "keeping already published target state for this run"
            )
            return
        self.scenario = scenario
        self.last_treatment_signature = ()
        self.stable_treatment_scan_count = 0
        self.scan_complete_sent = False
        self.get_logger().info(f"Scenario target filter active: {self.scenario}")

    # ------------------------------------------------------------------
    # Classification
    # ------------------------------------------------------------------

    def publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def lower_startswith_any(self, name: str, prefixes: List[str]) -> bool:
        lower = name.lower()
        return any(lower.startswith(prefix) for prefix in prefixes)

    def is_ignored(self, name: str) -> bool:
        return self.lower_startswith_any(name, self.ignored_prefixes)

    def is_agricultural_target(self, name: str) -> bool:
        if self.is_ignored(name):
            return False
        return self.lower_startswith_any(name, self.allowed_target_prefixes)

    def is_obstacle(self, name: str) -> bool:
        if self.is_ignored(name):
            return False
        if self.lower_startswith_any(name, self.obstacle_prefixes):
            return True
        # Trees/crops are also physical objects, but they are handled as service targets.
        return False

    def classify_target(self, name: str) -> str:
        lower = name.lower()
        if lower.startswith("tree_"):
            return "tree"
        if lower.startswith("crop_row"):
            return "crop"
        return "unknown"

    def obstacle_radius(self, name: str) -> float:
        lower = name.lower()
        for prefix, radius in self.obstacle_radius_by_prefix.items():
            if lower.startswith(prefix):
                return radius
        return self.default_obstacle_radius

    def analyze_crop_health(self, model_name: str, target_type: str) -> Dict[str, str]:
        """
        Simulated AI analysis stage.
        In a real system, replace with UAV image inference:
          - object detection / segmentation
          - disease classification
          - irrigation/fertilizer decision
        """
        if target_type == "tree":
            return {
                "needs_treatment": True,
                "status": "tree_inspection",
                "severity": "medium",
            }

        digest = hashlib.md5(model_name.encode()).hexdigest()
        score = int(digest[:2], 16) % 100

        if score < 55:
            return {"needs_treatment": False, "status": "healthy", "severity": "none"}
        if score < 70:
            return {"needs_treatment": True, "status": "water_stress", "severity": "medium"}
        if score < 85:
            return {"needs_treatment": True, "status": "fertilizer_need", "severity": "medium"}
        return {"needs_treatment": True, "status": "diseased", "severity": "high"}

    # ------------------------------------------------------------------
    # World parsing
    # ------------------------------------------------------------------

    def parse_world_models(self) -> List[Dict]:
        models = []

        if not self.world_file.exists():
            self.get_logger().error(f"World file not found: {self.world_file}")
            return models

        try:
            tree = ET.parse(self.world_file)
            root = tree.getroot()
        except Exception as exc:
            self.get_logger().error(f"Failed to parse world file: {exc}")
            return models

        for model in root.iter("model"):
            name = model.attrib.get("name", "")
            if not name or self.is_ignored(name):
                continue

            pose_element = model.find("pose")
            if pose_element is None or pose_element.text is None:
                continue

            values = pose_element.text.strip().split()
            if len(values) < 2:
                continue

            x = float(values[0]) + self.map_offset_x
            y = float(values[1]) + self.map_offset_y
            z = float(values[2]) if len(values) >= 3 else 0.0

            # Prefer live Gazebo pose when available. This allows moving animals/humans.
            if name in self.live_model_poses:
                x, y, z = self.live_model_poses[name]

            item = {
                "name": name,
                "x": x,
                "y": y,
                "z": z,
                "is_target": self.is_agricultural_target(name),
                "is_obstacle": self.is_obstacle(name),
                "radius": self.obstacle_radius(name),
            }

            if item["is_target"]:
                target_type = self.classify_target(name)
                analysis = self.analyze_crop_health(name, target_type)
                item.update({
                    "type": target_type,
                    "needs_treatment": analysis["needs_treatment"],
                    "status": analysis["status"],
                    "severity": analysis["severity"],
                })
            else:
                item.update({
                    "type": "obstacle" if item["is_obstacle"] else "other",
                    "needs_treatment": False,
                    "status": "obstacle" if item["is_obstacle"] else "other",
                    "severity": "none",
                })

            models.append(item)

        return models

    # ------------------------------------------------------------------
    # Publishing targets
    # ------------------------------------------------------------------

    def should_publish_target(self, target: Dict) -> bool:
        if not target.get("needs_treatment", False):
            return False
        allowed = self.scenario_target_names.get(self.scenario)
        if allowed is not None and target["name"] not in allowed:
            return False

        return target["name"] not in self.sent_targets

    def publish_target(self, target: Dict) -> bool:
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(target["x"])
        msg.pose.position.y = float(target["y"])
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0

        try:
            self.target_pub.publish(msg)
        except Exception as exc:
            self.get_logger().error(
                f"Failed to publish treatment target {target['name']}: {exc}"
            )
            return False

        self.sent_targets[target["name"]] = {
            "x": target["x"],
            "y": target["y"],
            "type": target["type"],
            "status": target["status"],
            "severity": target["severity"],
            "last_sent": time.time(),
        }

        self.get_logger().info(
            f"UAV sent treatment target: {target['name']} | "
            f"type={target['type']} | status={target['status']} | "
            f"severity={target['severity']} | x={target['x']:.2f}, y={target['y']:.2f}"
        )
        return True

    def publish_scan_complete(self, treatment_target_count: int):
        msg = UInt32()
        msg.data = treatment_target_count
        self.scan_complete_pub.publish(msg)
        self.scan_complete_sent = True
        self.publish_status(
            f"UAV scan complete: treatment_targets={treatment_target_count}"
        )

    # ------------------------------------------------------------------
    # Aerial map/image/markers
    # ------------------------------------------------------------------

    def world_to_grid(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        gx = int((x - self.min_x) / self.grid_resolution)
        gy = int((y - self.min_y) / self.grid_resolution)
        if gx < 0 or gy < 0 or gx >= self.grid_width or gy >= self.grid_height:
            return None
        return gx, gy

    def world_to_image(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        ix = int((x - self.min_x) / (self.max_x - self.min_x) * (self.image_width - 1))
        iy = int((self.max_y - y) / (self.max_y - self.min_y) * (self.image_height - 1))
        if ix < 0 or iy < 0 or ix >= self.image_width or iy >= self.image_height:
            return None
        return ix, iy

    def mark_grid_circle(self, data: List[int], x: float, y: float, radius: float, value: int):
        center = self.world_to_grid(x, y)
        if center is None:
            return
        cx, cy = center
        cells = max(1, int(radius / self.grid_resolution))
        for dy in range(-cells, cells + 1):
            for dx in range(-cells, cells + 1):
                if dx * dx + dy * dy > cells * cells:
                    continue
                gx = cx + dx
                gy = cy + dy
                if 0 <= gx < self.grid_width and 0 <= gy < self.grid_height:
                    data[gy * self.grid_width + gx] = value

    def publish_aerial_obstacle_map(self, models: List[Dict]):
        msg = OccupancyGrid()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = self.grid_resolution
        msg.info.width = self.grid_width
        msg.info.height = self.grid_height
        msg.info.origin.position.x = self.min_x
        msg.info.origin.position.y = self.min_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        data = [0] * (self.grid_width * self.grid_height)

        # Obstacles from aerial survey: rocks, fence, animals, humans.
        for item in models:
            if item["is_obstacle"]:
                self.mark_grid_circle(data, item["x"], item["y"], item["radius"], 100)

        # Mark crops/trees as softer obstacles for awareness.
        for item in models:
            if item["is_target"]:
                soft_radius = 0.35 if item["type"] == "crop" else 0.55
                self.mark_grid_circle(data, item["x"], item["y"], soft_radius, 40)

        msg.data = data
        self.map_pub.publish(msg)

    def draw_disc(self, image: bytearray, x: int, y: int, r: int, color: Tuple[int, int, int]):
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx * dx + dy * dy > r * r:
                    continue
                px = x + dx
                py = y + dy
                if 0 <= px < self.image_width and 0 <= py < self.image_height:
                    idx = (py * self.image_width + px) * 3
                    image[idx] = color[0]
                    image[idx + 1] = color[1]
                    image[idx + 2] = color[2]

    def draw_box(self, image: bytearray, x: int, y: int, half: int, color: Tuple[int, int, int]):
        for py in range(max(0, y - half), min(self.image_height, y + half + 1)):
            for px in range(max(0, x - half), min(self.image_width, x + half + 1)):
                idx = (py * self.image_width + px) * 3
                image[idx] = color[0]
                image[idx + 1] = color[1]
                image[idx + 2] = color[2]

    def publish_aerial_image(self, models: List[Dict]):
        # Green farm background
        image = bytearray([70, 110, 55] * (self.image_width * self.image_height))

        # Simple grid lines
        for px in range(0, self.image_width, 35):
            for py in range(self.image_height):
                idx = (py * self.image_width + px) * 3
                image[idx:idx + 3] = bytes([85, 130, 70])
        for py in range(0, self.image_height, 35):
            for px in range(self.image_width):
                idx = (py * self.image_width + px) * 3
                image[idx:idx + 3] = bytes([85, 130, 70])

        for item in models:
            p = self.world_to_image(item["x"], item["y"])
            if p is None:
                continue
            ix, iy = p

            if item["is_obstacle"]:
                # Obstacles: gray/brown/black
                self.draw_box(image, ix, iy, 9, (80, 80, 80))
            elif item["is_target"]:
                if item.get("needs_treatment", False):
                    # Diseased/treatment target: red
                    self.draw_disc(image, ix, iy, 7, (220, 40, 35))
                else:
                    # Healthy crop/tree: bright green
                    self.draw_disc(image, ix, iy, 5, (20, 180, 40))

        msg = Image()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = self.image_height
        msg.width = self.image_width
        msg.encoding = "rgb8"
        msg.is_bigendian = 0
        msg.step = self.image_width * 3
        msg.data = bytes(image)
        self.image_pub.publish(msg)

    def make_marker(self, marker_id: int, item: Dict) -> Marker:
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "uav_farm_survey"
        marker.id = marker_id
        marker.action = Marker.ADD
        marker.pose.position.x = float(item["x"])
        marker.pose.position.y = float(item["y"])
        marker.pose.position.z = 0.25
        marker.pose.orientation.w = 1.0

        marker.type = Marker.SPHERE
        marker.scale.x = 0.35
        marker.scale.y = 0.35
        marker.scale.z = 0.35

        if item["is_obstacle"]:
            marker.color.r = 0.15
            marker.color.g = 0.15
            marker.color.b = 0.15
            marker.color.a = 0.85
        elif item["is_target"] and item.get("needs_treatment", False):
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.95
        else:
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.70

        return marker

    def publish_markers(self, models: List[Dict]):
        arr = MarkerArray()
        marker_id = 0

        for item in models:
            if item["is_obstacle"] or item["is_target"]:
                arr.markers.append(self.make_marker(marker_id, item))
                marker_id += 1

        # UAV position marker
        uav = Marker()
        uav.header.frame_id = "map"
        uav.header.stamp = self.get_clock().now().to_msg()
        uav.ns = "uav_farm_survey"
        uav.id = marker_id
        uav.type = Marker.CUBE
        uav.action = Marker.ADD
        uav.pose.position.x = 0.0
        uav.pose.position.y = 0.0
        uav.pose.position.z = self.uav_altitude
        uav.pose.orientation.w = 1.0
        uav.scale.x = 0.6
        uav.scale.y = 0.6
        uav.scale.z = 0.15
        uav.color.r = 0.05
        uav.color.g = 0.05
        uav.color.b = 0.12
        uav.color.a = 0.9
        arr.markers.append(uav)

        self.marker_pub.publish(arr)

    def publish_uav_pose(self):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = self.uav_altitude
        pose.pose.orientation.w = 1.0
        self.uav_pose_pub.publish(pose)

    def publish_survey_report(self, models: List[Dict]):
        targets = []
        obstacles = []

        for item in models:
            if item["is_target"]:
                targets.append({
                    "name": item["name"],
                    "type": item["type"],
                    "x": round(item["x"], 3),
                    "y": round(item["y"], 3),
                    "needs_treatment": bool(item.get("needs_treatment", False)),
                    "status": item.get("status", "unknown"),
                    "severity": item.get("severity", "none"),
                })
            elif item["is_obstacle"]:
                obstacles.append({
                    "name": item["name"],
                    "x": round(item["x"], 3),
                    "y": round(item["y"], 3),
                    "radius": round(item.get("radius", 0.5), 3),
                    "status": "dynamic_or_static_obstacle",
                })

        report = {
            "stamp": time.time(),
            "wall_time_unix": time.time(),
            "survey_sim_time_s": self.get_clock().now().nanoseconds / 1e9,
            "frame_id": "map",
            "uav": {
                "name": "uav_scout_drone",
                "altitude": self.uav_altitude,
                "mission": "aerial_survey",
            },
            "targets": targets,
            "obstacles": obstacles,
            "summary": {
                "total_targets": len(targets),
                "needs_treatment": sum(1 for t in targets if t["needs_treatment"]),
                "obstacles": len(obstacles),
            },
        }

        msg = String()
        msg.data = json.dumps(report)
        self.survey_report_pub.publish(msg)

    # ------------------------------------------------------------------
    # Main survey loop
    # ------------------------------------------------------------------

    def scan_farm(self):
        now_sim = self.get_clock().now().nanoseconds / 1e9
        if self.survey_started_at is None:
            self.survey_started_at = now_sim
        elapsed = now_sim - self.survey_started_at

        if elapsed < self.takeoff_seconds:
            self.publish_status(f"UAV taking off... altitude target={self.uav_altitude:.1f}m")
            self.publish_uav_pose()
            return

        self.publish_status("UAV aerial survey running: capturing top-down farm image and coordinates")

        models = self.parse_world_models()
        self.publish_uav_pose()
        self.publish_aerial_obstacle_map(models)
        self.publish_aerial_image(models)
        self.publish_markers(models)
        self.publish_survey_report(models)

        targets = [m for m in models if m["is_target"]]
        allowed = self.scenario_target_names.get(self.scenario)
        treatment_targets = [
            m for m in targets
            if m.get("needs_treatment", False)
            and (allowed is None or m["name"] in allowed)
        ]
        obstacles = [m for m in models if m["is_obstacle"]]
        treatment_signature = tuple(
            sorted(target["name"] for target in treatment_targets)
        )

        if treatment_signature == self.last_treatment_signature:
            self.stable_treatment_scan_count += 1
        else:
            self.last_treatment_signature = treatment_signature
            self.stable_treatment_scan_count = 1

        self.get_logger().info(
            f"Aerial survey complete: targets={len(targets)}, "
            f"needs_treatment={len(treatment_targets)}, obstacles={len(obstacles)}"
        )

        for target in treatment_targets:
            self.get_logger().info(
                f"Inspecting treatment target: {target['name']} -> {target['status']} "
                f"({target['severity']}) x={target['x']:.2f}, y={target['y']:.2f}"
            )
            if self.should_publish_target(target):
                self.publish_target(target)
            else:
                self.get_logger().info(
                    f"Treatment target already sent: {target['name']}"
                )

        remaining = [
            target["name"]
            for target in treatment_targets
            if target["name"] not in self.sent_targets
        ]
        self.get_logger().info(
            f"Treatment publication progress: discovered={len(treatment_targets)}, "
            f"sent={len(treatment_targets) - len(remaining)}, "
            f"remaining={len(remaining)}, "
            f"stable_scans={self.stable_treatment_scan_count}/"
            f"{self.required_stable_treatment_scans}"
        )

        if (
            not self.scan_complete_sent
            and not remaining
            and self.stable_treatment_scan_count
            >= self.required_stable_treatment_scans
        ):
            self.publish_scan_complete(len(treatment_targets))


def main():
    rclpy.init()
    node = UAVFarmScanner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
