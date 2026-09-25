#!/usr/bin/env python3

import argparse
import math
import os
import time
from agri_mission.physical_time import PhysicalClock, PhysicalClockError
from typing import Dict, List, Optional, Set, Tuple

import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from geometry_msgs.msg import (
    PoseStamped,
    PointStamped,
    PoseWithCovarianceStamped,
    Twist,
)
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from lifecycle_msgs.srv import GetState
from std_srvs.srv import Empty
from action_msgs.msg import GoalStatus
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, UInt32
from tf2_ros import Buffer, TransformListener, TransformException

from agri_mission.target_manager import TargetManager

try:
    from agri_mission.ros2_evaluation_logger import EvaluationLogger
except ImportError:
    # Allows running the source file directly when the logger module is kept
    # beside this file during development.
    from ros2_evaluation_logger import EvaluationLogger


def yaw_to_quaternion(yaw: float) -> Tuple[float, float]:
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return qz, qw


class AgriMissionManager(BasicNavigator):
    TARGET_DISCOVERED = "DISCOVERED"
    TARGET_QUEUED = "QUEUED"
    TARGET_ACTIVE = "ACTIVE"
    TARGET_NAVIGATING = "NAVIGATING"
    TARGET_TREATED = "TREATED"
    TARGET_DEFERRED = "DEFERRED"
    TARGET_UNREACHABLE = "UNREACHABLE"
    TARGET_FAILED = "FAILED"

    """
    Persistent-target UGV mission manager.

    Mission rule:
      - The UGV must NOT skip to the next target just because one path failed.
      - The current target remains active until it is treated.
      - If the path is blocked, the robot stops, clears costmaps, replans from the current pose,
        and tries a new approach path to the SAME target.
      - Only after treatment is completed does it move to the next target.
      - Home is saved from AMCL or TF and is used only after all targets are completed.
    """

    def __init__(
        self,
        run_id: str = "manual",
        scenario: str = "S1",
        method: str = "P",
        seed: Optional[int] = None,
        log_path: str = "/tmp/uav_benchmark/event_log.jsonl",
        software_commit: str = "unknown",
    ):
        super().__init__(node_name="agri_mission_manager")
        try:
            self.set_parameters([
                Parameter("use_sim_time", Parameter.Type.BOOL, True)
            ])
        except Exception as exc:
            self.get_logger().warn(f"Could not enable use_sim_time on mission manager: {exc}")

        # ---------------- QUANTITATIVE EVALUATION ----------------
        self.eval_run_id = str(run_id)
        self.eval_scenario = str(scenario)
        self.eval_method = str(method)
        self.eval_seed = seed
        self.eval_log_path = str(log_path)
        self.eval_software_commit = str(software_commit)
        self.eval_logger = EvaluationLogger(
            self.eval_log_path,
            self.eval_run_id,
            self.eval_scenario,
            self.eval_method,
            self.eval_seed,
        )
        self.eval_target_order = 0
        self.eval_target_orders: Dict[str, int] = {}
        self.eval_active_target_id: Optional[str] = None
        self.eval_target_attempt = 0
        self.eval_active_candidate_id: Optional[str] = None
        self.eval_active_candidate_index: Optional[int] = None
        self.eval_active_candidate_angle: Optional[float] = None
        self.eval_active_candidate_radius: Optional[float] = None
        self.eval_active_goal_x: Optional[float] = None
        self.eval_active_goal_y: Optional[float] = None
        self.eval_nav_started_monotonic: Optional[float] = None
        self.eval_nav_started_sim: Optional[float] = None
        self.eval_last_progress_monotonic: Optional[float] = None
        self.eval_last_progress_x: Optional[float] = None
        self.eval_last_progress_y: Optional[float] = None
        self.eval_actual_path_length = 0.0
        self.eval_planning_time_accumulator = 0.0
        self.eval_recovery_started_monotonic: Optional[float] = None
        self.eval_recovery_type: Optional[str] = None
        self.eval_cancel_started_monotonic: Optional[float] = None
        self.eval_cancel_pending = False
        self.eval_mission_started_sim: Optional[float] = None
        self.eval_mission_started_monotonic: Optional[float] = None
        self.eval_mission_end_emitted = False
        self.eval_manual_intervention = False
        self.eval_collision_count = 0

        self.single_candidate_radius_m = 1.0
        self.single_candidate_angle_deg = 180

        # Shared Navigation v2 safety parameters (all benchmark methods).
        for name, default in {
            "map_boundary_margin_m": 0.75,
            "localization_max_age_s": 3.0,
            "localization_future_tolerance_s": 0.3,
            "return_home_localization_wait_s": 2.0,
            "localization_max_xy_variance": 0.5,
            "localization_max_yaw_variance": 0.5,
            "localization_tf_amcl_max_distance_m": 0.5,
            "localization_tf_amcl_max_yaw_rad": 0.5,
            "home_planning_timeout_s": 10.0,
            "max_return_home_attempts": 2,
            "max_return_home_replans": 1,
            "return_home_multistage": True,
            "return_home_stage_distance_m": 5.0,
            "dynamic_obstacle_observation_s": 2.0,
            "min_replan_interval_s": 3.0,
            # Bounded action/service watchdogs. These are infrastructure
            # safeguards, not method-selection parameters.
            "candidate_planning_timeout_s": 10.0,
            "planner_server_wait_timeout_s": 1.0,
            "planner_goal_response_timeout_s": 5.0,
            "planner_cancel_timeout_s": 3.0,
            "planner_response_cleanup_timeout_s": 10.0,
            "nav_action_server_wait_timeout_s": 3.0,
            "nav_goal_response_timeout_s": 8.0,
            "nav_response_cleanup_timeout_s": 20.0,
            # AMCL remains the global localization anchor. When an update is
            # motion-triggered but arrives slightly late, odometry propagation
            # bridges only this short bounded interval.
            "amcl_motion_stale_grace_s": 2.0,
        }.items():
            value = self.declare_parameter(name, default).value
            if not isinstance(default, bool) and (not math.isfinite(value) or value <= 0):
                raise ValueError(f"{name} must be finite and positive")
            setattr(self, name, value)
        self.last_amcl_diagnostic = {}
        self.latest_amcl_pose = None
        # Keep the last ACCEPTED AMCL fix separate from the latest received
        # message. Rejected/stale measurements must never reset odometry
        # baselines used by motion-aware freshness checks.
        self.last_valid_amcl_pose_msg = None
        self.last_valid_amcl_diagnostic = {}
        self.amcl_motion_stale_grace_started_wall = None
        self.localization_reason = "amcl_missing"

        # Planner/navigation submission diagnostics. Planner requests use local
        # action handles so they can never overwrite NavigateToPose state.
        self.last_planner_failure_reason = ""
        self.planner_submission_uncertain = False
        self.navigation_submission_uncertain = False
        self.last_candidate_batch_infrastructure_reason = ""
        self.infrastructure_fault_reason = ""
        self.max_infrastructure_retries_per_target = 2
        self.localization_consistency = {}
        self.last_localization_refresh_wall = -math.inf
        self.last_localization_refresh_sim = -math.inf
        self.last_navigation_terminal_status = "not_started"
        self.last_navigation_terminal_result = "not_started"
        self.last_navigation_terminal_reason = "not_started"
        self.return_odom_samples = 0
        self.return_odom_valid = True
        self.return_last_odom = None
        self.home_precheck_attempt = 0
        self.pre_home_recovery_used = False
        self.global_costmap_received_sim = None
        self.local_costmap = None
        self.local_costmap_received_sim = None
        self.blocked_start_recovery_pose = None
        self.lifecycle_observations = {}
        self.home_precheck_failure_reason = None
        self.static_map = None
        self.last_valid_amcl_sim = None
        self.last_valid_tf_sim = None
        self.home_stamp_sim = None
        self.home_source = None
        self.last_obstacle_replan_time = -math.inf
        self.last_navigation_submission_time = -math.inf
        self.return_distance = None
        self.return_last_xy = None

        # Continuous odometry motion state for deciding whether AMCL's
        # /request_nomotion_update service is safe to use.
        self.latest_odom_linear_mps = math.inf
        self.latest_odom_angular_rps = math.inf
        self.latest_odom_stamp = None
        self.odom_stationary_since_sim = None
        self.odom_stationary_linear_threshold_mps = 0.01
        self.odom_stationary_angular_threshold_rps = 0.01
        self.odom_stationary_required_s = 0.5

        # RC11: continuous odometry pose used to distinguish an AMCL message
        # that is old because the robot has not crossed AMCL motion thresholds
        # from a genuinely stale localization stream.
        self.latest_odom_x = None
        self.latest_odom_y = None
        self.latest_odom_yaw = None

        # Odometry pose captured when the most recent AMCL message is received.
        self.last_amcl_odom_x = None
        self.last_amcl_odom_y = None
        self.last_amcl_odom_yaw = None
        self.last_amcl_odom_stamp = None

        # Match the active AMCL motion-update thresholds.
        self.amcl_update_min_d_m = 0.25
        self.amcl_update_min_a_rad = 0.20

        # RC11: cumulative physical odometry motion. Net displacement is not
        # sufficient because the robot may move away and later return close
        # to the same pose.
        self.odom_motion_total_distance_m = 0.0
        self.odom_motion_total_rotation_rad = 0.0
        self.odom_motion_last_x = None
        self.odom_motion_last_y = None
        self.odom_motion_last_yaw = None

        # Cumulative-motion baselines corresponding to the latest AMCL message.
        self.last_amcl_odom_total_distance_m = None
        self.last_amcl_odom_total_rotation_rad = None

        # ---------------- HOME / LOCALIZATION ----------------
        self.home_x: Optional[float] = None
        self.home_y: Optional[float] = None
        self.home_yaw = 1.57
        self.home_saved = False

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.have_amcl_pose = False

        # ---------------- MISSION PARAMETERS ----------------
        # Close enough to visibly treat the plant without driving into its stem.
        self.treatment_reach_radius = 1.15

        # Prefer close service points now that crop collision no longer blocks
        # the whole row gap; keep a slightly wider fallback for tight cases.
        self.service_rings = [0.80, 1.00, 1.20]

        # Service point tolerance. This does NOT trigger treatment alone.
        self.service_acceptance_radius = 0.25

        self.home_acceptance_radius = 0.95
        self.treatment_seconds = 5.0
        self.duplicate_distance = 0.75

        # Target retry policy.
        # Keep working on the same target until it is treated.
        self.retry_same_target_until_treated = True
        self.replan_pause_sec = 0.35
        self.max_replan_rounds_per_cycle = 4
        self.max_candidates_checked = 32
        self.max_candidates_to_execute_per_cycle = 3
        self.max_target_candidate_cycles = 3
        self.max_consecutive_no_candidate_cycles = 3
        self.max_deferred_retry_passes = 2
        #self.deferred_retry_pass = 0
        self.last_target_failure_reason = ""
        self.current_candidate_rejections: List[str] = []

        # Navigation action lifecycle.
        self.cancel_timeout_sec = 20.0
        self.active_navigation_goal = False
        self.navigation_cancel_pending = False
        self.pending_recovery_reason: Optional[str] = None
        # After the primary cancellation timeout, allow one additional bounded
        # window for Nav2 to publish the terminal NavigateToPose result.
        self.pending_cancel_terminal_deadline_monotonic: Optional[float] = None
        self.navigation_lifecycle_fault = False
        self.mission_halted = False
        self.motion_state = "STOPPED"
        self.last_navigation_result = "none"
        self.last_navigation_reason = "not_started"
        self.manual_recovery_active = False
        self.stop_command_latched = False
        self.last_stop_diagnostic_time = 0.0
        self.stop_diagnostic_interval_sec = 2.0
        self.max_stop_burst_messages = 3

        # Navigation timing.
        self.min_candidate_timeout_sec = 45.0
        self.max_candidate_timeout_sec = 120.0
        self.seconds_per_meter = 5.0

        self.home_timeout_sec = 420.0
        self.home_retries = 2

        # Stuck detection for target approach.
        self.stuck_check_window_sec = 18.0
        self.min_progress_required = 0.07

        # Target-focused behavior:
        # If the robot is already near the active target, it should not drive far away.
        self.target_focus_radius = 3.0
        self.max_allowed_retreat_from_target = 1.2
        self.target_progress_window_sec = self.stuck_check_window_sec
        self.min_target_progress_required = 0.05

        # Costmap safety.
        self.global_costmap: Optional[OccupancyGrid] = None
        self.uav_aerial_map: Optional[OccupancyGrid] = None

        self.costmap_lethal_threshold = 90
        self.uav_blocked_threshold = 60

        self.costmap_unknown_is_unsafe = True
        self.uav_unknown_is_unsafe = False

        self.candidate_clearance_radius = 0.24
        self.path_clearance_radius = 0.12
        self.uav_candidate_clearance_radius = 0.45
        self.uav_path_clearance_radius = 0.25

        # RC2 physical robot footprint derived from URDF wheel/chassis collision geometry.
        # Keep this distinct from candidate/path/UAV safety clearances.
        self.physical_footprint = (
            (0.23, 0.22),
            (0.23, -0.22),
            (-0.27, -0.22),
            (-0.27, 0.22),
        )
        self.physical_footprint_front = 0.23
        self.physical_footprint_rear = -0.27
        self.physical_footprint_left = 0.22
        self.physical_footprint_right = -0.22

        # Laser / dynamic obstacle awareness.
        self.front_obstacle_detected = False
        self.eval_previous_front_obstacle_detected = False
        self.front_obstacle_distance = float("inf")
        self.left_obstacle_distance = float("inf")
        self.right_obstacle_distance = float("inf")
        self.front_obstacle_stop_distance = 0.42
        self.front_sector_degrees = 30.0
        self.front_block_cancel_after_sec = 1.2

        # Dynamic obstacle policy:
        # brief crossing = wait; persistent blocking = fast replan.
        self.dynamic_wait_before_replan_sec = 0.8
        self.dynamic_clear_confirm_sec = 0.35
        self.dynamic_obstacle_cancel_timeout_sec = 8.0

        # Escape recovery when the robot is trapped in a narrow/closed path.
        # The robot backs up, rotates away from the blocked side, then replans the SAME target.
        self.escape_backup_speed = -0.13
        self.escape_backup_sec = 1.75
        self.escape_turn_speed = 0.55
        self.escape_turn_sec = 1.45
        self.escape_forward_nudge_speed = 0.09
        self.escape_forward_nudge_sec = 0.45

        # Candidate-search escape policy. This is important when the robot is
        # stuck at the base of a tree/crop before a navigation task can start.
        self.max_path_rejections_before_escape = 3
        self.max_unsafe_candidates_before_escape = 18
        self.start_pose_recovery_count = 0
        self.max_start_pose_recoveries_per_cycle = 2
        self.escape_done_this_round = False

        # Explicit target-state ownership is delegated to TargetManager.
        self.target_manager = TargetManager()

        # Only /cmd_vel drives the robot in this setup.
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        scenario_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.scenario_pub = self.create_publisher(
            String,
            "/benchmark/scenario",
            scenario_qos,
        )
        self.scenario_timer = self.create_timer(1.0, self.publish_scenario)

        # TF fallback for saving true home if /amcl_pose is delayed.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Persistent localization refresh service client.
        # RC7: reuse one client instead of creating/destroying it on every request.
        self.nomotion_update_client = self.create_client(
            Empty,
            "/request_nomotion_update",
        )

        # ---------------- SUBSCRIBERS ----------------
        target_qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        scan_complete_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        costmap_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            PoseStamped,
            "/uav/tree_target",
            self.pose_target_callback,
            target_qos,
        )
        self.create_subscription(
            UInt32,
            "/uav/scan_complete",
            self.scan_complete_callback,
            scan_complete_qos,
        )
        self.create_subscription(PointStamped, "/clicked_point", self.clicked_point_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.amcl_pose_callback, costmap_qos)
        self.create_subscription(Odometry, "/odom", self.return_odom_callback, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, "/map", self.static_map_callback, costmap_qos)
        self.create_subscription(OccupancyGrid, "/local_costmap/costmap", self.local_costmap_callback, costmap_qos)
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap", self.global_costmap_callback, costmap_qos)
        self.create_subscription(OccupancyGrid, "/uav/aerial_obstacle_map", self.uav_aerial_map_callback, 10)
        self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info("Agri Mission Manager is ready")
        if self.eval_method == "P":
            self.get_logger().info("Persistent target mode enabled: target will not be skipped until treated")
        else:
            self.get_logger().info(
                f"Baseline method enabled: {self.eval_method}; failed targets "
                "terminate cleanly without P-style persistent retry"
            )
        self.get_logger().info("Waiting for UAV targets on /uav/tree_target")
        self.get_logger().info("Using UAV aerial obstacle map /uav/aerial_obstacle_map for candidate/path rejection")
        self.publish_scenario()

    def eval_sim_time(self) -> float:
        """Return ROS/simulation time in seconds for quantitative logging."""
        try:
            return self.get_clock().now().nanoseconds / 1.0e9
        except Exception:
            return float("nan")

    def publish_scenario(self):
        msg = String()
        msg.data = self.eval_scenario
        self.scenario_pub.publish(msg)

    def eval_emit(self, event: str, **fields):
        """Emit structured JSONL data without allowing logging to stop the mission."""
        try:
            # Preserve legacy fields and add explicit clock provenance.
            if "planning_time_s" in fields:
                fields["planning_wall_latency_s"] = fields["planning_time_s"]
            if "home_planning_time_s" in fields:
                fields["home_planning_wall_latency_s"] = fields["home_planning_time_s"]
            if "navigation_time_s" in fields:
                key = "return_navigation_sim_time_s" if event == "return_home_end" else "navigation_sim_time_s"
                fields[key] = fields["navigation_time_s"]
            self.eval_logger.emit(event, sim_time_s=self.eval_sim_time(), **fields)
        except Exception as exc:
            self.get_logger().error(f"Evaluation logging failed for {event}: {exc}")

    def eval_target_context(self) -> dict:
        return {
            "target_id": self.eval_active_target_id,
            "attempt": self.eval_target_attempt,
            "candidate_id": self.eval_active_candidate_id,
        }

    def eval_clear_nav_tracking(self):
        self.eval_active_candidate_id = None
        self.eval_active_candidate_index = None
        self.eval_active_candidate_angle = None
        self.eval_active_candidate_radius = None
        self.eval_active_goal_x = None
        self.eval_active_goal_y = None
        self.eval_nav_started_monotonic = None
        self.eval_nav_started_sim = None
        self.eval_last_progress_monotonic = None
        self.eval_last_progress_x = None
        self.eval_last_progress_y = None
        self.eval_actual_path_length = 0.0

    def eval_emit_mission_end(self, mission_completed: bool, returned_home: bool, manual_intervention: bool = False):
        if self.eval_mission_end_emitted:
            return
        self.eval_mission_end_emitted = True
        total_time = None
        if self.eval_mission_started_sim is not None:
            total_time = self.eval_sim_time() - self.eval_mission_started_sim
        self.eval_emit(
            "mission_end",
            mission_completed=bool(mission_completed),
            returned_home=bool(returned_home),
            manual_intervention=bool(manual_intervention),
            collision_count=self.eval_collision_count,
            total_mission_time_s=total_time,
            mission_sim_time_s=total_time,
            mission_wall_time_s=(time.monotonic()-self.eval_mission_started_monotonic
                                 if self.eval_mission_started_monotonic is not None else None),
        )

    # ------------------------------------------------------------------
    # BASIC HELPERS
    # ------------------------------------------------------------------

    def spin_for_duration(self, duration):
        """Physical dwell in ROS time; executor waits remain bounded wall waits."""
        deadline = self.physical_now() + duration
        while rclpy.ok() and self.physical_now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not rclpy.ok():
            raise PhysicalClockError("shutdown_during_physical_dwell")

    def physical_now(self):
        if not hasattr(self, "physical_clock"):
            self.physical_clock = PhysicalClock(self.eval_sim_time)
        return self.physical_clock.read()

    def spin_wall_duration(self, duration):
        """Infrastructure polling, independent of simulation clock progression."""
        deadline = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=min(0.05, max(0.0, deadline-time.monotonic())))

    def observe_lifecycle(self, name):
        client = self.create_client(GetState, f"{name}/get_state")
        try:
            if not client.wait_for_service(timeout_sec=0.5):
                return None
            future = client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            label = future.result().current_state.label if future.done() and future.exception() is None else None
            self.lifecycle_observations[name] = {"label": label, "sim_time_s": self.eval_sim_time()}
            return label
        finally:
            self.destroy_client(client)

    def _waitForInitialPose(self):
        """Bound BasicNavigator's otherwise unbounded AMCL startup wait."""
        deadline = time.monotonic() + 30.0
        while rclpy.ok() and not self.initial_pose_received:
            self._setInitialPose()
            rclpy.spin_once(self, timeout_sec=0.2)
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    "Timed out waiting for the initial AMCL pose"
                )

    def _waitForNodeToActivate(self, node_name):
        # Humble BasicNavigator unconditionally sleeps two seconds even after ACTIVE.
        # That can age out the only initial AMCL fix before our callback runs.
        deadline = time.monotonic()+90.0
        while rclpy.ok() and time.monotonic() < deadline:
            if self.observe_lifecycle(node_name) == "active":
                return
            self.spin_wall_duration(0.1)
        raise TimeoutError(f"Startup lifecycle activation timed out: {node_name}")

    def navigation_diagnostic_snapshot(self):
        finite = lambda value: value if value is not None and math.isfinite(value) else None
        now = self.eval_sim_time()
        cell = self.world_to_costmap_index(self.current_x, self.current_y)
        cost = self.costmap_cell_value(*cell) if cell is not None else None
        footprint_cost = self.footprint_max_cost(
            self.global_costmap,
            self.current_x,
            self.current_y,
            self.current_yaw,
        ) if self.global_costmap is not None else None
        edge = None
        if self.static_map is not None:
            info = self.static_map.info
            q = info.origin.orientation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            dx,dy=self.current_x-info.origin.position.x,self.current_y-info.origin.position.y
            gx,gy=math.cos(yaw)*dx+math.sin(yaw)*dy,-math.sin(yaw)*dx+math.cos(yaw)*dy
            edge = min(gx,gy,info.width*info.resolution-gx,info.height*info.resolution-gy)
        local_cost = None
        local_footprint_cost = None
        if self.local_costmap is not None:
            try:
                tf = self.tf_buffer.lookup_transform(self.local_costmap.header.frame_id,"base_footprint",rclpy.time.Time())
                index = self.world_to_grid_index(self.local_costmap,tf.transform.translation.x,tf.transform.translation.y)
                local_cost = self.grid_cell_value(self.local_costmap,*index) if index is not None else None
                q = tf.transform.rotation
                local_yaw = math.atan2(
                    2 * (q.w * q.z + q.x * q.y),
                    1 - 2 * (q.y * q.y + q.z * q.z),
                )
                local_footprint_cost = self.footprint_max_cost(
                    self.local_costmap,
                    tf.transform.translation.x,
                    tf.transform.translation.y,
                    local_yaw,
                )
            except TransformException:
                pass
        return {"robot_x":self.current_x,"robot_y":self.current_y,"robot_yaw":self.current_yaw,
                "amcl_age_s":now-self.last_amcl_diagnostic["stamp"] if self.last_amcl_diagnostic else None,
                "localization_reason":self.localization_reason, "localization_consistency":self.localization_consistency,
                "tf_age_s":now-self.last_valid_tf_sim if self.last_valid_tf_sim is not None else None,
                "amcl":self.last_amcl_diagnostic,"lifecycle":self.lifecycle_observations,
                "global_costmap_age_s":now-self.global_costmap_received_sim if self.global_costmap_received_sim is not None else None,
                "local_costmap_age_s":now-self.local_costmap_received_sim if self.local_costmap_received_sim is not None else None,
                "global_cell_cost":cost,"global_footprint_max_cost":footprint_cost,"local_cell_cost":local_cost, "local_footprint_max_cost":local_footprint_cost,
                "map_edge_distance_m":edge,"front_obstacle":self.front_obstacle_detected,
                "nearest_scan_obstacle_m":finite(getattr(self,"nearest_scan_obstacle_distance",None)),
                "active_goal":self.active_navigation_goal,"cancel_pending":self.navigation_cancel_pending,
                "manual_recovery":self.manual_recovery_active,"motion_state":self.motion_state,
                "current_target":self.current_target["id"] if self.current_target else None,
                "previous_navigation_result":self.last_navigation_result,"previous_navigation_reason":self.last_navigation_reason}

    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        self.amcl_callback_serial = getattr(self, "amcl_callback_serial", 0) + 1
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        q = msg.pose.pose.orientation
        values = [msg.pose.pose.position.x, msg.pose.pose.position.y,
                  q.x, q.y, q.z, q.w, *msg.pose.covariance]
        now_sim = self.eval_sim_time()

        # RC11 startup bootstrap: a TRANSIENT_LOCAL AMCL pose can arrive
        # before this node has received its first /clock sample. In that case
        # sim time is still zero and timestamp validation must be deferred.
        clock_ready = now_sim > 0.0
        age = now_sim - stamp if clock_ready else 0.0

        reason = None
        if msg.header.frame_id != "map":
            reason = "wrong_frame"
        elif not all(math.isfinite(v) for v in values):
            reason = "nonfinite_pose_or_covariance"
        elif (
            clock_ready
            and not -self.localization_future_tolerance_s
                    <= age
                    <= self.localization_max_age_s
        ):
            reason = "pose_timestamp_out_of_range"
        elif any(not -1e-9 <= msg.pose.covariance[i] <= self.localization_max_xy_variance for i in (0, 7)) or not -1e-9 <= msg.pose.covariance[35] <= self.localization_max_yaw_variance:
            reason = "invalid_covariance"
        elif abs(sum(v*v for v in (q.x, q.y, q.z, q.w))-1.0) > 0.05:
            reason = "invalid_quaternion"
        self.last_amcl_diagnostic = {
            "stamp": stamp, "age_at_receipt_s": age, "frame": msg.header.frame_id,
            "covariance_xy_yaw": [v if math.isfinite(v) else None for v in (msg.pose.covariance[0], msg.pose.covariance[7], msg.pose.covariance[35])],
            "rejection_reason": reason}
        self.latest_amcl_pose = msg

        if reason:
            # IMPORTANT: a rejected AMCL message is diagnostic evidence only.
            # It must NOT reset the odometry baseline associated with the last
            # trusted global localization fix.
            self.eval_emit("localization_check", **self.last_amcl_diagnostic)
            return

        self.last_valid_amcl_sim = stamp
        self.last_valid_amcl_pose_msg = msg
        self.last_valid_amcl_diagnostic = dict(self.last_amcl_diagnostic)
        self.amcl_motion_stale_grace_started_wall = None

        # Snapshot odometry only for an ACCEPTED AMCL fix.
        if all(v is not None for v in (
            self.latest_odom_x,
            self.latest_odom_y,
            self.latest_odom_yaw,
            self.latest_odom_stamp,
        )):
            self.last_amcl_odom_x = self.latest_odom_x
            self.last_amcl_odom_y = self.latest_odom_y
            self.last_amcl_odom_yaw = self.latest_odom_yaw
            self.last_amcl_odom_stamp = self.latest_odom_stamp
            self.last_amcl_odom_total_distance_m = (
                self.odom_motion_total_distance_m
            )
            self.last_amcl_odom_total_rotation_rad = (
                self.odom_motion_total_rotation_rad
            )

        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.have_amcl_pose = True

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        # Home is captured only after the joint AMCL/TF policy passes.

    def return_odom_callback(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        linear = msg.twist.twist.linear
        angular = msg.twist.twist.angular

        linear_speed = math.sqrt(
            linear.x * linear.x
            + linear.y * linear.y
            + linear.z * linear.z
        )
        angular_speed = math.sqrt(
            angular.x * angular.x
            + angular.y * angular.y
            + angular.z * angular.z
        )

        pose = msg.pose.pose
        q = pose.orientation
        odom_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

        if all(math.isfinite(v) for v in (
            stamp,
            linear_speed,
            angular_speed,
            pose.position.x,
            pose.position.y,
            odom_yaw,
        )):
            self.latest_odom_stamp = stamp
            self.latest_odom_linear_mps = linear_speed
            self.latest_odom_angular_rps = angular_speed
            self.latest_odom_x = pose.position.x
            self.latest_odom_y = pose.position.y
            self.latest_odom_yaw = odom_yaw

            # RC11 startup bootstrap: AMCL may arrive before the first odom
            # sample. Attach the first valid odom pose to that existing AMCL
            # message so motion-aware staleness can be evaluated normally.
            if (
                self.last_valid_amcl_pose_msg is not None
                and self.last_amcl_odom_x is None
                and self.last_amcl_odom_y is None
                and self.last_amcl_odom_yaw is None
            ):
                self.last_amcl_odom_x = pose.position.x
                self.last_amcl_odom_y = pose.position.y
                self.last_amcl_odom_yaw = odom_yaw
                self.last_amcl_odom_stamp = stamp
                self.last_amcl_odom_total_distance_m = (
                    self.odom_motion_total_distance_m
                )
                self.last_amcl_odom_total_rotation_rad = (
                    self.odom_motion_total_rotation_rad
                )

            if (
                self.odom_motion_last_x is not None
                and self.odom_motion_last_y is not None
                and self.odom_motion_last_yaw is not None
            ):
                step_distance = math.hypot(
                    pose.position.x - self.odom_motion_last_x,
                    pose.position.y - self.odom_motion_last_y,
                )
                step_rotation = abs(math.atan2(
                    math.sin(odom_yaw - self.odom_motion_last_yaw),
                    math.cos(odom_yaw - self.odom_motion_last_yaw),
                ))

                if math.isfinite(step_distance) and math.isfinite(step_rotation):
                    self.odom_motion_total_distance_m += step_distance
                    self.odom_motion_total_rotation_rad += step_rotation

            self.odom_motion_last_x = pose.position.x
            self.odom_motion_last_y = pose.position.y
            self.odom_motion_last_yaw = odom_yaw

            stationary = (
                linear_speed < self.odom_stationary_linear_threshold_mps
                and angular_speed < self.odom_stationary_angular_threshold_rps
            )

            if stationary:
                if self.odom_stationary_since_sim is None:
                    self.odom_stationary_since_sim = stamp
            else:
                self.odom_stationary_since_sim = None

        if self.return_distance is None:
            return

        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        frame = (msg.header.frame_id, msg.child_frame_id)
        if not all(math.isfinite(v) for v in (stamp, x, y)) or not all(frame):
            self.return_odom_valid = False
            return
        if stamp < self.return_phase_start_sim:
            return  # Queued odometry preceding the return phase is not return travel.
        previous = self.return_last_odom
        if previous is None:
            self.return_first_odom_sim = stamp
            if stamp-self.return_phase_start_sim > 1.0:
                self.return_odom_valid = False
        if previous is not None:
            dt = stamp-previous[0]
            if dt == 0:
                return
            distance = math.hypot(x-previous[1], y-previous[2])
            # Reject missing intervals/frame resets/implausible jumps, not real travel.
            if dt < 0 or dt > 1.0 or frame != previous[3] or distance > 2.0*dt+0.05:
                self.return_odom_valid = False
            else:
                self.return_distance += distance
        self.return_last_odom = (stamp, x, y, frame)
        self.return_odom_samples += 1

    def try_update_pose_from_tf(self, *, amcl_tf_validated=False) -> bool:
        if not amcl_tf_validated:
            return self.localization_valid()
        try:
            tf = self.tf_buffer.lookup_transform(
                "map",
                "base_footprint",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
        except TransformException:
            return False

        stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
        if not -self.localization_future_tolerance_s <= self.eval_sim_time()-stamp <= self.localization_max_age_s:
            return False
        if not all(math.isfinite(v) for v in (tf.transform.translation.x, tf.transform.translation.y)):
            return False
        self.last_valid_tf_sim = stamp
        self.current_x = tf.transform.translation.x
        self.current_y = tf.transform.translation.y

        q = tf.transform.rotation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        return True

    def get_current_pose(self) -> Optional[PoseStamped]:
        """Return the robot pose in map frame using TF/AMCL cached pose."""
        if not self.localization_valid():
            return None

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.current_x
        pose.pose.position.y = self.current_y
        pose.pose.position.z = 0.0

        qz = math.sin(self.current_yaw / 2.0)
        qw = math.cos(self.current_yaw / 2.0)
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        return pose

    def is_pose_inside_static_map_bounds(self, x: float, y: float) -> bool:
        """Startup-only bounds check that does not depend on Nav2 costmap arrival."""
        grid = self.static_map
        if (
            grid is None
            or grid.header.frame_id != "map"
            or grid.info.resolution <= 0.0
        ):
            return False

        info = grid.info
        q = info.origin.orientation
        grid_yaw = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y * q.y + q.z * q.z),
        )
        dx = x - info.origin.position.x
        dy = y - info.origin.position.y
        gx = math.cos(grid_yaw) * dx + math.sin(grid_yaw) * dy
        gy = -math.sin(grid_yaw) * dx + math.cos(grid_yaw) * dy
        margin = self.map_boundary_margin_m
        return (
            margin <= gx < info.width * info.resolution - margin
            and margin <= gy < info.height * info.resolution - margin
        )

    def save_home_from_tf_if_available(self) -> bool:
        if self.home_saved:
            return True

        # Home is a localization/static-map concept. Nav2 global costmap is
        # explicitly awaited AFTER home capture, avoiding a circular startup
        # dependency on a costmap that may not have arrived yet.
        if self.localization_valid() and self.is_pose_inside_static_map_bounds(
            self.current_x, self.current_y
        ):
            self.home_x, self.home_y, self.home_yaw = self.current_x, self.current_y, self.current_yaw
            self.home_saved = True
            self.home_stamp_sim = self.eval_sim_time()
            self.home_source = "validated_amcl_map_pose"
            return True
        return False

    def static_map_callback(self, msg: OccupancyGrid):
        self.static_map = msg

    def odom_motion_since_last_amcl(self):
        """Return odometry pose change since the latest AMCL message.

        AMCL update_min_d/update_min_a are motion thresholds relative to the
        odometry pose associated with the previous filter update, not cumulative
        path length travelled.
        """
        if any(v is None for v in (
            self.latest_odom_x,
            self.latest_odom_y,
            self.latest_odom_yaw,
            self.last_amcl_odom_x,
            self.last_amcl_odom_y,
            self.last_amcl_odom_yaw,
        )):
            return None, None

        distance = math.hypot(
            self.latest_odom_x - self.last_amcl_odom_x,
            self.latest_odom_y - self.last_amcl_odom_y,
        )

        rotation = abs(math.atan2(
            math.sin(self.latest_odom_yaw - self.last_amcl_odom_yaw),
            math.cos(self.latest_odom_yaw - self.last_amcl_odom_yaw),
        ))

        if not all(math.isfinite(v) for v in (distance, rotation)):
            return None, None

        return distance, rotation

    def propagated_amcl_pose_from_odom(self):
        """Propagate the latest AMCL map pose using relative odometry motion.

        Used only when the AMCL message is legitimately old because the robot
        has not crossed AMCL's configured motion-update thresholds.
        """
        msg = self.last_valid_amcl_pose_msg

        if msg is None or any(v is None for v in (
            self.latest_odom_x,
            self.latest_odom_y,
            self.latest_odom_yaw,
            self.last_amcl_odom_x,
            self.last_amcl_odom_y,
            self.last_amcl_odom_yaw,
        )):
            return None

        aq = msg.pose.pose.orientation
        amcl_yaw = math.atan2(
            2.0 * (aq.w * aq.z + aq.x * aq.y),
            1.0 - 2.0 * (aq.y * aq.y + aq.z * aq.z),
        )

        dx_odom = self.latest_odom_x - self.last_amcl_odom_x
        dy_odom = self.latest_odom_y - self.last_amcl_odom_y

        c0 = math.cos(self.last_amcl_odom_yaw)
        s0 = math.sin(self.last_amcl_odom_yaw)

        # Relative translation expressed in the robot frame at the AMCL update.
        rel_x = c0 * dx_odom + s0 * dy_odom
        rel_y = -s0 * dx_odom + c0 * dy_odom

        ca = math.cos(amcl_yaw)
        sa = math.sin(amcl_yaw)

        predicted_x = (
            msg.pose.pose.position.x
            + ca * rel_x
            - sa * rel_y
        )
        predicted_y = (
            msg.pose.pose.position.y
            + sa * rel_x
            + ca * rel_y
        )

        delta_yaw = math.atan2(
            math.sin(self.latest_odom_yaw - self.last_amcl_odom_yaw),
            math.cos(self.latest_odom_yaw - self.last_amcl_odom_yaw),
        )

        predicted_yaw = math.atan2(
            math.sin(amcl_yaw + delta_yaw),
            math.cos(amcl_yaw + delta_yaw),
        )

        if not all(math.isfinite(v) for v in (
            predicted_x,
            predicted_y,
            predicted_yaw,
        )):
            return None

        return predicted_x, predicted_y, predicted_yaw

    def amcl_motion_update_expected(self):
        """True when odometry motion since the latest AMCL message is large
        enough that AMCL should have produced a motion-triggered update.
        Unknown odometry state is treated conservatively as requiring an update.
        """
        distance, rotation = self.odom_motion_since_last_amcl()

        if distance is None or rotation is None:
            return True

        return (
            distance >= self.amcl_update_min_d_m
            or rotation >= self.amcl_update_min_a_rad
        )

    def localization_valid(self):
        """Validate AMCL as the global anchor with a bounded odometry bridge.

        Fresh accepted AMCL fixes are checked against time-aligned map->base TF.
        If AMCL is older only because no motion update is required, the accepted
        fix is propagated with odometry. If a motion-triggered AMCL update is
        expected, the same propagation is allowed only for a short bounded grace
        interval; after that the state becomes ``amcl_stale``.

        Rejected AMCL measurements (high covariance, wrong frame, non-finite
        values, invalid quaternion) are never used as a new propagation anchor.
        """
        reason = "valid"
        now_sim = self.eval_sim_time()
        now_wall = time.monotonic()
        diag = self.last_amcl_diagnostic
        msg = self.last_valid_amcl_pose_msg
        valid_stamp = self.last_valid_amcl_sim

        amcl_age = (
            now_sim - valid_stamp
            if valid_stamp is not None
            else math.inf
        )
        motion_distance, motion_rotation = self.odom_motion_since_last_amcl()
        motion_update_expected = self.amcl_motion_update_expected()
        latest_rejection = diag.get("rejection_reason")

        self.localization_consistency = {
            "amcl_age_s": amcl_age if math.isfinite(amcl_age) else None,
            "odom_motion_since_amcl_m": motion_distance,
            "odom_rotation_since_amcl_rad": motion_rotation,
            "amcl_motion_update_expected": motion_update_expected,
        }

        # A newly received measurement with a structural/confidence failure is
        # a real localization fault. Timestamp-only rejection is handled using
        # the last accepted AMCL fix below.
        if latest_rejection not in (None, "pose_timestamp_out_of_range"):
            reason = {
                "invalid_covariance": "amcl_covariance_high",
            }.get(latest_rejection, "amcl_" + latest_rejection)
        elif msg is None or valid_stamp is None:
            reason = "amcl_missing"
        elif amcl_age < -self.localization_future_tolerance_s:
            reason = "amcl_stale"
        else:
            stale = amcl_age > self.localization_max_age_s
            motion_grace = False

            if stale and motion_update_expected:
                if self.amcl_motion_stale_grace_started_wall is None:
                    self.amcl_motion_stale_grace_started_wall = now_wall

                grace_elapsed = (
                    now_wall - self.amcl_motion_stale_grace_started_wall
                )
                motion_grace = (
                    grace_elapsed <= self.amcl_motion_stale_grace_s
                )
                self.localization_consistency.update({
                    "amcl_motion_stale_grace_elapsed_s": grace_elapsed,
                    "amcl_motion_stale_grace_limit_s":
                        self.amcl_motion_stale_grace_s,
                })

                if not motion_grace:
                    reason = "amcl_stale"
            else:
                self.amcl_motion_stale_grace_started_wall = None

            if reason == "valid" and stale:
                # AMCL remains the global anchor; odometry only propagates that
                # accepted fix until the next valid AMCL update.
                propagated = self.propagated_amcl_pose_from_odom()
                if propagated is None:
                    reason = "amcl_stale"
                else:
                    predicted_x, predicted_y, predicted_yaw = propagated
                    self.current_x = predicted_x
                    self.current_y = predicted_y
                    self.current_yaw = predicted_yaw
                    self.have_amcl_pose = True
                    self.localization_consistency.update({
                        "stale_amcl_propagated_with_odom": True,
                        "motion_update_grace_active": bool(motion_grace),
                    })

            elif reason == "valid":
                # Fresh accepted AMCL fix: verify it against map->base TF at
                # the same timestamp, then update the live pose from latest TF.
                try:
                    aligned = self.tf_buffer.lookup_transform(
                        "map",
                        "base_footprint",
                        rclpy.time.Time.from_msg(msg.header.stamp),
                    )

                    q = aligned.transform.rotation
                    aq = msg.pose.pose.orientation
                    yaw = math.atan2(
                        2 * (q.w*q.z + q.x*q.y),
                        1 - 2 * (q.y*q.y + q.z*q.z),
                    )
                    ayaw = math.atan2(
                        2 * (aq.w*aq.z + aq.x*aq.y),
                        1 - 2 * (aq.y*aq.y + aq.z*aq.z),
                    )

                    distance = math.hypot(
                        aligned.transform.translation.x
                        - msg.pose.pose.position.x,
                        aligned.transform.translation.y
                        - msg.pose.pose.position.y,
                    )
                    angle = abs(math.atan2(
                        math.sin(yaw - ayaw),
                        math.cos(yaw - ayaw),
                    ))

                    self.localization_consistency.update({
                        "tf_amcl_distance_m": distance,
                        "tf_amcl_yaw_rad": angle,
                    })

                    if (
                        not all(math.isfinite(v) for v in (distance, angle))
                        or distance
                        > self.localization_tf_amcl_max_distance_m
                        or angle > self.localization_tf_amcl_max_yaw_rad
                    ):
                        reason = "tf_amcl_mismatch"
                    elif not self.try_update_pose_from_tf(
                        amcl_tf_validated=True
                    ):
                        reason = "tf_stale"

                except TransformException:
                    reason = "tf_missing"

        if reason != self.localization_reason:
            self.eval_emit(
                "localization_check",
                check_type="validity",
                reason=reason,
                amcl=diag,
                **self.localization_consistency,
            )

        self.localization_reason = reason
        return reason == "valid"

    def drain_localization_tf_callbacks(self):
        """Bound transport/executor arrival skew, without relaxing AMCL acceptance."""
        valid = self.localization_valid()
        if valid or self.localization_reason not in ("tf_missing", "tf_stale"):
            return valid
        deadline = time.monotonic() + 0.5
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.01)
            valid = self.localization_valid()
            if valid or self.localization_reason not in ("tf_missing", "tf_stale"):
                break
        self.eval_emit("localization_check", check_type="tf_callback_wait",
                       valid=valid, reason=self.localization_reason)
        return valid

    def request_localization_update(self):
        """One bounded request; acknowledgement is not a sensor-pose acceptance."""
        self.last_localization_refresh_wall = time.monotonic()
        self.last_localization_refresh_sim = self.eval_sim_time()
        before_serial = getattr(self, "amcl_callback_serial", 0)
        before_stamp = self.last_amcl_diagnostic.get("stamp", -math.inf)
        client = self.nomotion_update_client
        sent = acknowledged = callback_received = accepted = False
        if client.wait_for_service(timeout_sec=1.0):
            future = client.call_async(Empty.Request())
            sent = True
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            acknowledged = future.done() and future.exception() is None
            deadline = time.monotonic() + self.return_home_localization_wait_s
            while rclpy.ok() and time.monotonic() < deadline:
                callback_received = (getattr(self, "amcl_callback_serial", 0) > before_serial
                                     and self.last_amcl_diagnostic.get("stamp", -math.inf) > before_stamp)
                if callback_received:
                    accepted = self.drain_localization_tf_callbacks()
                    break
                rclpy.spin_once(self, timeout_sec=0.01)
        self.eval_emit("localization_check", check_type="nomotion_update",
                       request_sent=sent, request_acknowledged=acknowledged,
                       fresh_amcl_callback_received=callback_received,
                       pose_accepted=accepted, reason=self.localization_reason,
                       amcl=self.last_amcl_diagnostic)
        return accepted

    def wait_for_valid_localization(self):
        # Treatment is blocking in the preserved mission architecture. Drain
        # queued clock/TF callbacks before assessing age at a home stage boundary.
        valid = self.localization_valid()

        # RC11: a no-motion request is only a bootstrap for a completely
        # missing AMCL stream. Do not force an AMCL measurement update for
        # covariance, TF, or motion-triggered staleness failures.
        if not valid and self.latest_amcl_pose is None:
            self.request_localization_update()
        deadline = time.monotonic() + self.return_home_localization_wait_s
        valid_since = None
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.localization_valid():
                valid_since = time.monotonic() if valid_since is None else valid_since
                if time.monotonic()-valid_since >= 0.1:
                    return True
            else:
                valid_since = None
        return False

    def is_pose_inside_navigable_bounds(self, x, y):
        if not all(math.isfinite(v) for v in (x, y)):
            return False
        for grid in (self.static_map, self.global_costmap):
            if grid is None or grid.header.frame_id != "map" or grid.info.resolution <= 0:
                return False
            info = grid.info
            q = info.origin.orientation
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
            dx, dy = x-info.origin.position.x, y-info.origin.position.y
            gx, gy = math.cos(yaw)*dx + math.sin(yaw)*dy, -math.sin(yaw)*dx + math.cos(yaw)*dy
            margin = self.map_boundary_margin_m
            if not (margin <= gx < info.width*info.resolution-margin
                    and margin <= gy < info.height*info.resolution-margin):
                return False
        return True

    def manual_translation_safe(self, twist):
        if not self.localization_valid():
            return False

        if self.global_costmap is None:
            return False

        x, y, yaw = self.current_x, self.current_y, self.current_yaw
        start_cost = self.footprint_max_cost(self.global_costmap, x, y, yaw)
        if start_cost is None or not math.isfinite(float(start_cost)):
            return False
        start_cost = float(start_cost)

        # Normal manual translation remains conservative and unchanged in spirit.
        if start_cost < self.costmap_lethal_threshold:
            for _ in range(41):
                if (
                    not self.is_pose_inside_navigable_bounds(x, y)
                    or not self.is_physical_footprint_safe(
                        self.global_costmap, x, y, yaw
                    )
                    or not self.is_point_uav_aerial_safe(
                        x, y, clearance_radius=0.0
                    )
                ):
                    return False
                x += twist.linear.x * math.cos(yaw) * 0.05
                y += twist.linear.x * math.sin(yaw) * 0.05
                yaw += twist.angular.z * 0.05
            return True

        # Egress exception: when inflation/inscribed cost already marks the
        # footprint blocked, the ordinary predicate would make escape
        # mathematically impossible. Only REVERSE egress is permitted.
        if twist.linear.x >= -1.0e-6:
            return False

        rear = getattr(self, "rear_obstacle_distance", float("inf"))
        rear_required = getattr(self, "rear_backup_min_clearance", 0.75)
        if not math.isfinite(rear) or rear < rear_required:
            return False

        # Never use this exception when the physical footprint already
        # intersects a truly lethal/static obstacle cell.
        lethal = self.exact_physical_footprint_has_lethal_cell(
            self.global_costmap, x, y, yaw, 100
        )
        if lethal is None or lethal:
            return False

        best_cost = start_cost
        for _ in range(41):
            if not self.is_pose_inside_navigable_bounds(x, y):
                return False
            if not self.is_point_uav_aerial_safe(x, y, clearance_radius=0.0):
                return False
            cost = self.footprint_max_cost(self.global_costmap, x, y, yaw)
            if cost is None or not math.isfinite(float(cost)):
                return False
            cost = float(cost)
            if cost >= 100.0 or cost > start_cost + 1.0e-6:
                return False
            best_cost = min(best_cost, cost)
            x += twist.linear.x * math.cos(yaw) * 0.05
            y += twist.linear.x * math.sin(yaw) * 0.05
            yaw += twist.angular.z * 0.05

        final_cost = self.footprint_max_cost(self.global_costmap, x, y, yaw)
        return (
            final_cost is not None
            and math.isfinite(float(final_cost))
            and float(final_cost) < start_cost
            and best_cost < start_cost
        )

    def global_costmap_callback(self, msg: OccupancyGrid):
        self.global_costmap = msg
        self.global_costmap_received_sim = self.eval_sim_time()

    def local_costmap_callback(self, msg: OccupancyGrid):
        self.local_costmap = msg
        self.local_costmap_received_sim = self.eval_sim_time()

    def wait_for_global_costmap(self, timeout_sec: float = 20.0) -> bool:
        start = time.monotonic()
        while rclpy.ok() and self.global_costmap is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() - start >= timeout_sec:
                break

        if self.global_costmap is not None:
            info = self.global_costmap.info
            self.get_logger().info(
                f"Global costmap ready from topic: {info.width}x{info.height}, "
                f"resolution={info.resolution:.3f}"
            )
            return True

        # Do not call BasicNavigator.getGlobalCostmap() here: the Humble
        # helper waits without a deadline and returns nav2_msgs/Costmap rather
        # than the OccupancyGrid used by this manager's safety routines.
        self.get_logger().error(
            "Global OccupancyGrid was not received before mission_start; "
            "mission cannot safely begin"
        )
        return False

    def uav_aerial_map_callback(self, msg: OccupancyGrid):
        self.uav_aerial_map = msg

    def scan_callback(self, msg: LaserScan):
        front_half = math.radians(self.front_sector_degrees)

        front_min = float("inf")
        left_min = float("inf")
        right_min = float("inf")
        rear_min = float("inf")

        angle = msg.angle_min
        for r in msg.ranges:
            if math.isfinite(r) and msg.range_min <= r <= msg.range_max:
                # Normalize every scan ray to [-pi, +pi].
                # This works whether the LaserScan is published as
                # -pi..+pi or 0..2*pi.
                a = math.atan2(math.sin(angle), math.cos(angle))

                # Front sector around 0 rad.
                if -front_half <= a <= front_half:
                    front_min = min(front_min, r)

                # Left side sector: +30 to +110 degrees.
                if math.radians(30.0) <= a <= math.radians(110.0):
                    left_min = min(left_min, r)

                # Right side sector: -110 to -30 degrees.
                if math.radians(-110.0) <= a <= math.radians(-30.0):
                    right_min = min(right_min, r)

                # Rear sector: +/-150..180 degrees.
                # This is required before ANY reverse recovery motion.
                if abs(a) >= math.radians(150.0):
                    rear_min = min(rear_min, r)

            angle += msg.angle_increment

        self.nearest_scan_obstacle_distance = min(
            (
                r for r in msg.ranges
                if math.isfinite(r) and msg.range_min <= r <= msg.range_max
            ),
            default=None,
        )
        self.front_obstacle_distance = front_min
        self.left_obstacle_distance = left_min
        self.right_obstacle_distance = right_min
        self.rear_obstacle_distance = rear_min

        # Conservative rear clearance for manual reverse recovery.
        # The robot footprint extends behind base_footprint, so backing up is
        # forbidden unless there is substantially more room than the body itself.
        self.rear_backup_min_clearance = 0.75

        self.front_obstacle_detected = front_min < self.front_obstacle_stop_distance
        if self.front_obstacle_detected and not self.eval_previous_front_obstacle_detected:
            self.eval_emit(
                "obstacle_detected", target_id=self.eval_active_target_id,
                obstacle_type="front_laser_obstacle", robot_x=float(self.current_x),
                robot_y=float(self.current_y), minimum_distance_m=float(front_min),
            )
        self.eval_previous_front_obstacle_detected = self.front_obstacle_detected

    def make_pose(self, x: float, y: float, yaw: float = 0.0) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0

        qz, qw = yaw_to_quaternion(yaw)
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def distance_to(self, x: float, y: float) -> float:
        return math.hypot(x - self.current_x, y - self.current_y)

    @property
    def target_queue(self) -> List[Dict]:
        return self.target_manager.target_queue

    @property
    def current_target(self) -> Optional[Dict]:
        return self.target_manager.current_target

    @current_target.setter
    def current_target(self, value: Optional[Dict]):
        self.target_manager.current_target = value

    @property
    def completed_targets(self) -> List[Dict]:
        return self.target_manager.completed_targets

    @property
    def deferred_targets(self) -> List[Dict]:
        return self.target_manager.deferred_targets

    @property
    def permanently_unreachable_targets(self) -> List[Dict]:
        return self.target_manager.permanently_unreachable_targets

    @property
    def failed_targets(self) -> List[Dict]:
        return self.target_manager.failed_targets

    @property
    def targets_by_id(self) -> Dict[str, Dict]:
        return self.target_manager.targets_by_id

    @property
    def uav_scan_complete(self) -> bool:
        return self.target_manager.uav_scan_complete

    @property
    def expected_uav_target_count(self) -> Optional[int]:
        return self.target_manager.expected_uav_target_count

    @expected_uav_target_count.setter
    def expected_uav_target_count(self, value: Optional[int]):
        self.target_manager.expected_uav_target_count = value

    @property
    def deferred_retry_pass(self) -> int:
        return self.target_manager.deferred_retry_pass

    @deferred_retry_pass.setter
    def deferred_retry_pass(self, value: int):
        self.target_manager.deferred_retry_pass = value

    @property
    def return_home_attempted(self) -> bool:
        return self.target_manager.return_home_attempted

    @return_home_attempted.setter
    def return_home_attempted(self, value: bool):
        self.target_manager.return_home_attempted = value

    @property
    def mission_summary_reported(self) -> bool:
        return self.target_manager.mission_summary_reported

    @mission_summary_reported.setter
    def mission_summary_reported(self, value: bool):
        self.target_manager.mission_summary_reported = value

    def safe_publish_twist(self, twist: Twist) -> bool:
        moving = any(
            abs(value) > 1.0e-6
            for value in (
                twist.linear.x,
                twist.linear.y,
                twist.linear.z,
                twist.angular.x,
                twist.angular.y,
                twist.angular.z,
            )
        )

        if self.active_navigation_goal:
            now = time.monotonic()
            if (
                now - self.last_stop_diagnostic_time
                >= self.stop_diagnostic_interval_sec
            ):
                self.last_stop_diagnostic_time = now
                self.get_logger().warn(
                    "Mission Twist suppressed because NavigateToPose is active; "
                    "Nav2 remains the only velocity authority"
                )
            return False

        if moving and not self.manual_recovery_active:
            self.get_logger().error(
                "Manual movement command suppressed outside MANUAL_RECOVERY"
            )
            return False

        if abs(twist.linear.x) > 1e-6 and not self.manual_translation_safe(twist):
            self.eval_emit("recovery_suppressed", reason="map_boundary_or_unsafe_space",
                           diagnostics=self.navigation_diagnostic_snapshot())
            self.cmd_vel_pub.publish(Twist())
            return False

        try:
            if rclpy.ok():
                self.cmd_vel_pub.publish(twist)
                if moving:
                    self.stop_command_latched = False
                return True
        except Exception:
            return False

        return False

    def stop_robot(self, repeats: int = 3) -> bool:
        if self.active_navigation_goal:
            now = time.monotonic()
            if (
                now - self.last_stop_diagnostic_time
                >= self.stop_diagnostic_interval_sec
            ):
                self.last_stop_diagnostic_time = now
                self.get_logger().warn(
                    "Hard stop suppressed while NavigateToPose is active; "
                    "cancel and confirm the action before publishing mission velocity"
                )
            return False

        if self.stop_command_latched:
            now = time.monotonic()
            if (
                now - self.last_stop_diagnostic_time
                >= self.stop_diagnostic_interval_sec
            ):
                self.last_stop_diagnostic_time = now
                self.get_logger().info(
                    "Duplicate hard stop suppressed because the robot is already stopped"
                )
            return True

        stop_msg = Twist()
        burst_count = max(1, min(repeats, self.max_stop_burst_messages))

        for _ in range(burst_count):
            self.safe_publish_twist(stop_msg)
            if rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.04)

        self.stop_command_latched = True
        self.motion_state = "STOPPED"
        self.last_stop_diagnostic_time = time.monotonic()
        self.get_logger().warn(
            f"Sending one bounded hard stop burst: messages={burst_count}"
        )
        return True

    def begin_manual_recovery(self, reason: str) -> bool:
        if (
            self.active_navigation_goal
            or self.navigation_cancel_pending
            or self.navigation_submission_uncertain
            or self.planner_submission_uncertain
        ):
            self.get_logger().error(
                "Cannot enter MANUAL_RECOVERY while planner/navigation "
                "transport is active, canceling, or unresolved"
            )
            return False

        if self.manual_recovery_active:
            self.get_logger().warn(
                "Duplicate MANUAL_RECOVERY entry suppressed"
            )
            return False

        self.manual_recovery_active = True
        self.motion_state = "MANUAL_RECOVERY"
        self.stop_command_latched = False
        self.eval_recovery_started_monotonic = time.monotonic()
        self.eval_recovery_started_sim = self.eval_sim_time()
        self.eval_recovery_type = "manual_recovery"
        self.eval_emit(
            "recovery_started",
            target_id=self.eval_active_target_id, attempt=self.eval_target_attempt,
            recovery_type=self.eval_recovery_type, reason=reason,
            front_clearance_m=(self.front_obstacle_distance if math.isfinite(self.front_obstacle_distance) else None),
            left_clearance_m=(self.left_obstacle_distance if math.isfinite(self.left_obstacle_distance) else None),
            right_clearance_m=(self.right_obstacle_distance if math.isfinite(self.right_obstacle_distance) else None),
            start_cost=self.current_costmap_status()[1],
        )
        self.get_logger().warn(
            f"Entering MANUAL_RECOVERY: {reason}"
        )
        return True

    def end_manual_recovery(self, stop_repeats: int = 3):
        self.stop_robot(repeats=stop_repeats)
        duration = time.monotonic() - (self.eval_recovery_started_monotonic or time.monotonic())
        self.manual_recovery_active = False
        self.motion_state = "STOPPED"
        self.eval_emit(
            "recovery_completed",
            target_id=self.eval_active_target_id, attempt=self.eval_target_attempt,
            recovery_type=self.eval_recovery_type or "manual_recovery",
            recovery_duration_s=duration,
            recovery_wall_duration_s=duration,
            recovery_sim_duration_s=self.eval_sim_time()-self.eval_recovery_started_sim,
        )
        self.eval_recovery_started_monotonic = None
        self.eval_recovery_type = None
        self.get_logger().info("MANUAL_RECOVERY completed")

    def wait_for_front_obstacle_clear(self, label: str) -> bool:
        wait_timeout = max(
            self.dynamic_wait_before_replan_sec,
            self.front_block_cancel_after_sec,
        )
        deadline = self.physical_now() + wait_timeout
        clear_since: Optional[float] = None

        while rclpy.ok() and self.physical_now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

            if self.front_obstacle_detected:
                clear_since = None
            else:
                if clear_since is None:
                    clear_since = self.physical_now()
                elif (
                    self.physical_now() - clear_since
                    >= self.dynamic_clear_confirm_sec
                ):
                    self.get_logger().info(
                        f"Obstacle cleared during OBSTACLE_WAIT for {label}; "
                        "changing candidate"
                    )
                    return True

            time.sleep(0.02)

        return False

    def controlled_backup(
        self,
        duration_sec: float = 0.45,
        speed: float = -0.04,
    ) -> bool:
        if not self.front_obstacle_detected:
            return False

        rear = getattr(self, "rear_obstacle_distance", float("inf"))
        rear_required = getattr(self, "rear_backup_min_clearance", 0.75)

        if not math.isfinite(rear) or rear < rear_required:
            self.get_logger().warn(
                f"Controlled backup suppressed: rear clearance={rear:.2f} m, "
                f"required>={rear_required:.2f} m"
            )
            self.stop_robot()
            return False

        if not self.begin_manual_recovery("controlled backup"):
            return False

        self.get_logger().warn(
            f"Front obstacle at {self.front_obstacle_distance:.2f} m. "
            "Executing small backup before replanning."
        )

        twist = Twist()
        twist.linear.x = speed

        start = self.physical_now()
        while rclpy.ok() and self.physical_now() - start < duration_sec:
            self.safe_publish_twist(twist)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.04)

        self.end_manual_recovery()
        return True

    def choose_escape_turn_direction(self) -> float:
        """
        Return angular.z sign for escape turn.
        Positive = turn left, negative = turn right.
        If left side is more open than right, turn left; otherwise turn right.
        """
        left = self.left_obstacle_distance
        right = self.right_obstacle_distance

        if not math.isfinite(left):
            left = 99.0
        if not math.isfinite(right):
            right = 99.0

        if left >= right:
            return abs(self.escape_turn_speed)
        return -abs(self.escape_turn_speed)

    def escape_recovery(self, reason: str = "blocked path", fast: bool = False) -> bool:
        """
        Recovery for narrow/closed paths:
          1) Stop
          2) Back up
          3) Turn toward the more open side
          4) Small forward nudge
          5) Stop and let Nav2 replan same target
        """
        if not self.begin_manual_recovery(reason):
            return False

        turn_z = self.choose_escape_turn_direction()
        turn_name = "left" if turn_z > 0 else "right"
        backup_speed = -0.28 if fast else self.escape_backup_speed
        backup_sec = 1.85 if fast else self.escape_backup_sec
        turn_speed = 0.85 if fast else self.escape_turn_speed
        turn_sec = 0.55 if fast else self.escape_turn_sec
        stop_repeats = 1 if fast else 3
        turn_z = math.copysign(turn_speed, turn_z)

        rear = getattr(self, "rear_obstacle_distance", float("inf"))
        rear_required = getattr(self, "rear_backup_min_clearance", 0.75)
        rear_safe = math.isfinite(rear) and rear >= rear_required

        self.get_logger().warn(
            f"Escape recovery started because {reason}. "
            f"front={self.front_obstacle_distance:.2f}, "
            f"rear={rear:.2f}, "
            f"left={self.left_obstacle_distance:.2f}, "
            f"right={self.right_obstacle_distance:.2f}. "
            f"rear_safe={rear_safe}, turn={turn_name}, fast={fast}"
        )

        self.stop_robot(repeats=stop_repeats)

        # 1) Reverse ONLY when the rear LiDAR sector confirms adequate clearance.
        backup_origin = (self.current_x, self.current_y)
        backup_command_published = False
        if rear_safe:
            twist = Twist()
            twist.linear.x = backup_speed
            twist.angular.z = -0.35 * turn_z
            start = self.physical_now()

            while rclpy.ok() and self.physical_now() - start < backup_sec:
                rclpy.spin_once(self, timeout_sec=0.02)
                rear_now = getattr(self, "rear_obstacle_distance", float("inf"))
                if not math.isfinite(rear_now) or rear_now < rear_required:
                    self.get_logger().warn(
                        f"Backup stopped immediately: rear obstacle now {rear_now:.2f} m"
                    )
                    break
                if self.safe_publish_twist(twist):
                    backup_command_published = True
                time.sleep(0.04)

            self.stop_robot(repeats=stop_repeats)
        else:
            self.get_logger().warn(
                "Reverse portion of recovery suppressed because the rear "
                "LiDAR safety sector is blocked or unknown."
            )

        rclpy.spin_once(self, timeout_sec=0.05)
        backup_displacement = math.hypot(
            self.current_x - backup_origin[0], self.current_y - backup_origin[1]
        )
        if not backup_command_published or backup_displacement < 0.03:
            self.get_logger().warn(
                f"Recovery reverse produced no verified egress motion "
                f"(displacement={backup_displacement:.3f} m)."
            )
            self.end_manual_recovery(stop_repeats=stop_repeats)
            return False

        # If reverse motion improved the situation but the footprint remains in
        # inflation/inscribed cost, end this bounded egress step here. Rotating
        # while still blocked can sweep the body deeper into the obstacle.
        start_safe_after_backup, _ = self.current_costmap_status()
        if not start_safe_after_backup:
            self.get_logger().warn(
                "Reverse egress moved the robot but start footprint remains "
                "blocked; deferring rotation and allowing another bounded "
                "reverse egress step."
            )
            self.end_manual_recovery(stop_repeats=stop_repeats)
            return True

        # 2) Rotate only if the footprint sweep is safe.
        rotation_safe = self.is_rotation_sweep_safe(
            self.current_x,
            self.current_y,
            self.current_yaw,
            turn_z,
            turn_sec,
        )

        if not rotation_safe:
            opposite_turn = -turn_z
            opposite_safe = self.is_rotation_sweep_safe(
                self.current_x,
                self.current_y,
                self.current_yaw,
                opposite_turn,
                turn_sec,
            )

            if opposite_safe:
                turn_z = opposite_turn
                turn_name = "left" if turn_z > 0 else "right"
                self.get_logger().warn(
                    f"Preferred recovery turn unsafe; switching to {turn_name}"
                )
            else:
                self.get_logger().warn(
                    "Recovery suppressed: neither left nor right rotation "
                    "has a safe footprint sweep."
                )
                self.end_manual_recovery(stop_repeats=stop_repeats)
                return False

        # 3) Turn toward the verified safe side.
        twist = Twist()
        twist.angular.z = turn_z
        start = self.physical_now()
        while rclpy.ok() and self.physical_now() - start < turn_sec:
            self.safe_publish_twist(twist)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.04)

        self.stop_robot(repeats=stop_repeats)

        # 3) Small forward nudge after turning, only if front is not immediately blocked.
        if not fast and not self.front_obstacle_detected:
            twist = Twist()
            twist.linear.x = self.escape_forward_nudge_speed
            start = self.physical_now()
            while rclpy.ok() and self.physical_now() - start < self.escape_forward_nudge_sec:
                self.safe_publish_twist(twist)
                rclpy.spin_once(self, timeout_sec=0.02)
                time.sleep(0.04)

        self.end_manual_recovery(stop_repeats=stop_repeats)
        self.get_logger().warn("Manual escape recovery finished. Replanning SAME target.")
        return True

    def halt_mission_for_navigation_fault(self, reason: str):
        self.navigation_lifecycle_fault = True
        self.mission_halted = True
        self.get_logger().error(
            f"Navigation lifecycle failure: {reason}. "
            "Mission halted with the current target preserved; no new goal will be sent."
        )
        self.stop_robot()

    def cancel_active_navigation_and_wait(
        self,
        label: str,
        timeout_sec: Optional[float] = None,
    ) -> bool:
        """
        Request cancellation and wait for the active NavigateToPose result to
        become terminal. The cancellation response and terminal result share
        one bounded timeout.
        """
        if not self.active_navigation_goal:
            self.get_logger().info(
                f"No active navigation goal requires cancellation for {label}"
            )
            return self.wait_until_nav2_idle(label)

        if self.result_future is None or self.goal_handle is None:
            self.halt_mission_for_navigation_fault(
                f"active goal state is incomplete while canceling {label}"
            )
            return False

        if self.isTaskComplete():
            return self.wait_until_nav2_idle(label)

        self.get_logger().info(
            f"Requesting cancellation for active navigation goal: {label}"
        )
        self.motion_state = "CANCELING"
        self.navigation_cancel_pending = True

        cancel_timeout = (
            self.cancel_timeout_sec if timeout_sec is None else float(timeout_sec)
        )
        self.eval_cancel_started_monotonic = time.monotonic()
        self.eval_cancel_pending = True
        self.eval_emit(
            "cancel_requested",
            target_id=self.eval_active_target_id, attempt=self.eval_target_attempt,
            candidate_id=self.eval_active_candidate_id, reason=label,
            cancel_timeout_s=cancel_timeout,
        )
        deadline = time.monotonic() + cancel_timeout

        try:
            cancel_future = self.goal_handle.cancel_goal_async()
        except Exception as exc:
            self.halt_mission_for_navigation_fault(
                f"could not request cancellation for {label}: {exc}"
            )
            return False

        while rclpy.ok() and not cancel_future.done():
            if time.monotonic() >= deadline:
                self.get_logger().warn(
                    f"cancellation response timed out after "
                    f"{cancel_timeout:.1f}s for {label}; keeping the current "
                    "target ACTIVE and waiting for Nav2 to become terminal "
                    "before any manual recovery or new goal"
                )
                self.pending_recovery_reason = label
                self.pending_cancel_terminal_deadline_monotonic = (
                    time.monotonic() + cancel_timeout
                )
                self.eval_emit(
                    "cancel_terminal",
                    target_id=self.eval_active_target_id, attempt=self.eval_target_attempt,
                    candidate_id=self.eval_active_candidate_id, result_status="TIMEOUT",
                    cancel_duration_s=time.monotonic() - (self.eval_cancel_started_monotonic or time.monotonic()),
                )
                self.eval_cancel_pending = False
                return False

            rclpy.spin_once(self, timeout_sec=0.05)

        if not rclpy.ok():
            self.halt_mission_for_navigation_fault(
                f"ROS shutdown occurred while canceling {label}"
            )
            return False

        try:
            cancel_response = cancel_future.result()
        except Exception as exc:
            self.halt_mission_for_navigation_fault(
                f"cancellation request failed for {label}: {exc}"
            )
            return False

        goals_canceling = getattr(cancel_response, "goals_canceling", [])
        self.get_logger().info(
            f"Cancellation response received for {label}: "
            f"goals_canceling={len(goals_canceling)}"
        )

        while rclpy.ok() and time.monotonic() < deadline:
            if self.wait_until_nav2_idle(label, timeout_sec=0.0):
                self.navigation_cancel_pending = False
                return True

            rclpy.spin_once(self, timeout_sec=0.05)

        self.get_logger().warn(
            f"navigation goal did not reach a terminal state within "
            f"{cancel_timeout:.1f}s while canceling {label}; keeping the "
            "current target ACTIVE and waiting for Nav2 before recovery"
        )
        self.pending_recovery_reason = label
        self.pending_cancel_terminal_deadline_monotonic = (
            time.monotonic() + cancel_timeout
        )
        return False

    def capture_navigation_terminal(self, result, reason):
        # Use the action response before any live handle/status cleanup.
        status = self.status
        future = self.result_future
        if future is not None and future.done() and future.exception() is None:
            response = future.result()
            if response is not None:
                status = response.status
        if status is None:
            if self.last_navigation_terminal_status != "not_started":
                return  # An idle cleanup must not erase the prior terminal snapshot.
            status = "unavailable_no_terminal_result"
        names = {GoalStatus.STATUS_SUCCEEDED:"SUCCEEDED", GoalStatus.STATUS_ABORTED:"FAILED",
                 GoalStatus.STATUS_CANCELED:"CANCELED"}
        self.last_navigation_terminal_status = status
        self.last_navigation_terminal_result = names.get(status, getattr(result, "name", str(result)))
        self.last_navigation_terminal_reason = reason

    def wait_until_nav2_idle(
        self,
        label: str = "navigation",
        timeout_sec: float = 0.0,
    ) -> bool:
        deadline = time.monotonic() + max(0.0, timeout_sec)

        while rclpy.ok():
            task_complete = False
            try:
                task_complete = self.isTaskComplete()
            except Exception:
                task_complete = False

            result_done = self.result_future is None or getattr(
                self.result_future,
                "done",
                lambda: True,
            )()

            if self.active_navigation_goal and task_complete:
                result = self.getResult()
                self.capture_navigation_terminal(result, label)
                self.get_logger().info(
                    f"NavigateToPose terminal for {label}: result={result}"
                )
                if self.eval_cancel_pending:
                    self.eval_emit(
                        "cancel_terminal",
                        target_id=self.eval_active_target_id, attempt=self.eval_target_attempt,
                        candidate_id=self.eval_active_candidate_id, result_status=str(result),
                        cancel_duration_s=time.monotonic() - (self.eval_cancel_started_monotonic or time.monotonic()),
                    )
                    self.eval_cancel_pending = False
                self.active_navigation_goal = False
                self.navigation_cancel_pending = False
                self.motion_state = "STOPPED"
                self.goal_handle = None
                self.result_future = None
                self.status = None
                return True

            if (
                not self.active_navigation_goal
                and result_done
                and self.goal_handle is None
            ):
                self.navigation_cancel_pending = False
                self.motion_state = "STOPPED"
                return True

            if (
                not self.active_navigation_goal
                and result_done
                and self.goal_handle is not None
            ):
                self.get_logger().info(
                    f"Clearing stale NavigateToPose goal handle for {label}"
                )
                self.goal_handle = None
                self.result_future = None
                self.status = None
                self.navigation_cancel_pending = False
                self.motion_state = "STOPPED"
                return True

            if time.monotonic() >= deadline:
                self.get_logger().warn(
                    f"Nav2 is not idle for {label}: "
                    f"active_goal={self.active_navigation_goal}, "
                    f"goal_handle_present={self.goal_handle is not None}, "
                    f"result_done={result_done}"
                )
                return False

            rclpy.spin_once(self, timeout_sec=0.05)

        return False

    def finish_pending_cancel_recovery(self) -> bool:
        if not self.navigation_cancel_pending:
            return False

        # Once an unresolved action has been classified as a lifecycle fault,
        # keep ownership of it but do not repeatedly wait/log or issue new
        # recovery motion / navigation goals.
        if self.navigation_lifecycle_fault and self.mission_halted:
            return True

        if not self.wait_until_nav2_idle("pending cancellation", timeout_sec=0.0):
            now = time.monotonic()
            deadline = self.pending_cancel_terminal_deadline_monotonic

            if deadline is not None and now >= deadline:
                reason = (
                    self.pending_recovery_reason
                    or "pending navigation cancel"
                )

                if self.eval_cancel_pending:
                    self.eval_emit(
                        "cancel_terminal",
                        target_id=self.eval_active_target_id,
                        attempt=self.eval_target_attempt,
                        candidate_id=self.eval_active_candidate_id,
                        result_status="UNRESOLVED",
                        cancel_duration_s=now - (
                            self.eval_cancel_started_monotonic or now
                        ),
                    )
                    self.eval_cancel_pending = False

                self.pending_cancel_terminal_deadline_monotonic = None
                self.halt_mission_for_navigation_fault(
                    f"NavigateToPose cancellation for {reason} remained "
                    "non-terminal after the bounded terminal grace period"
                )
                return True

            remaining = (
                max(0.0, deadline - now)
                if deadline is not None
                else None
            )

            if remaining is None:
                self.get_logger().warn(
                    "Waiting for pending NavigateToPose cancellation to become "
                    "terminal; suppressing mission recovery velocity and new goals"
                )
            else:
                self.get_logger().warn(
                    "Waiting for pending NavigateToPose cancellation to become "
                    f"terminal; bounded grace remaining={remaining:.1f}s; "
                    "suppressing mission recovery velocity and new goals"
                )
            return True

        self.pending_cancel_terminal_deadline_monotonic = None
        reason = self.pending_recovery_reason or "pending navigation cancel"
        self.pending_recovery_reason = None

        # Localization cancellations are not physical entrapment. Manual
        # backup/rotation after AMCL/TF staleness can move an otherwise safe
        # robot into vegetation, so suppress escape motion for those reasons.
        localization_reasons = {
            "amcl_stale",
            "amcl_missing",
            "amcl_covariance_high",
            "tf_stale",
            "tf_missing",
            "tf_amcl_mismatch",
        }
        if reason in localization_reasons:
            self.get_logger().warn(
                f"Pending cancellation reached terminal state after {reason}; "
                "manual escape recovery suppressed. Waiting for localization "
                "and replanning from the current pose."
            )
            self.wait_for_valid_localization()
            return True

        self.get_logger().info(
            "Pending cancellation reached terminal state. Current target "
            "remains ACTIVE; executing obstacle/stall recovery before replanning."
        )

        self.post_nav2_cancel_recovery(reason, fast=True)
        return True

    def post_nav2_cancel_recovery(self, reason: str, fast: bool = False) -> bool:
        if not self.wait_until_nav2_idle(reason, timeout_sec=0.0):
            self.pending_recovery_reason = reason
            self.navigation_cancel_pending = True
            self.get_logger().warn(
                f"Post-cancel recovery delayed for {reason}; Nav2 is not idle yet"
            )
            return False

        self.get_logger().warn(
            f"Post-cancel recovery for {reason}: Nav2 is idle, clearing "
            "costmaps before manual backup/rotate"
        )
        self.clear_costmaps_safe()

        if not self.escape_recovery(reason=reason, fast=fast):
            return False

        self.clear_costmaps_safe()
        return True

    def clear_costmaps_safe(self):
        """Clear both Nav2 costmaps with hard wall-clock bounds."""
        self.get_logger().warn("Clearing Nav2 costmaps before replanning")
        local_ok = self.clear_local_costmap_bounded()
        global_ok = self.clear_global_costmap_bounded()
        if not (local_ok and global_ok):
            self.get_logger().warn(
                f"Bounded costmap clear incomplete: "
                f"local_ok={local_ok}, global_ok={global_ok}"
            )
        self.spin_wall_duration(0.15)
        return local_ok and global_ok

    def world_to_costmap_index(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        if self.global_costmap is None:
            return None

        return self.world_to_grid_index(self.global_costmap, x, y)

    def costmap_cell_value(self, mx: int, my: int) -> int:
        info = self.global_costmap.info
        idx = my * info.width + mx
        return self.global_costmap.data[idx]

    def world_to_grid_index(self, grid: OccupancyGrid, x: float, y: float) -> Optional[Tuple[int, int]]:
        info = grid.info
        if info.resolution <= 0 or not all(math.isfinite(v) for v in (x, y)):
            return None
        q = info.origin.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
        dx, dy = x-info.origin.position.x, y-info.origin.position.y
        mx = math.floor((math.cos(yaw)*dx + math.sin(yaw)*dy) / info.resolution)
        my = math.floor((-math.sin(yaw)*dx + math.cos(yaw)*dy) / info.resolution)

        if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
            return None

        return mx, my

    def grid_cell_value(self, grid: OccupancyGrid, mx: int, my: int) -> int:
        info = grid.info
        idx = my * info.width + mx
        return grid.data[idx]

    def is_point_in_grid_safe(
        self,
        grid: OccupancyGrid,
        x: float,
        y: float,
        clearance_radius: float,
        blocked_threshold: int,
        unknown_is_unsafe: bool,
    ) -> bool:
        info = grid.info
        center = self.world_to_grid_index(grid, x, y)

        if center is None:
            return False

        cx, cy = center
        cells_radius = max(1, math.ceil(clearance_radius / info.resolution))

        for dy in range(-cells_radius, cells_radius + 1):
            for dx in range(-cells_radius, cells_radius + 1):
                if dx * dx + dy * dy > cells_radius * cells_radius:
                    continue

                mx = cx + dx
                my = cy + dy

                if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
                    return False

                value = self.grid_cell_value(grid, mx, my)

                if value == -1 and unknown_is_unsafe:
                    return False

                if value >= blocked_threshold:
                    return False

        return True

    def is_point_uav_aerial_safe(
        self,
        x: float,
        y: float,
        clearance_radius: Optional[float] = None,
    ) -> bool:
        if clearance_radius is None:
            clearance_radius = self.uav_candidate_clearance_radius

        if self.uav_aerial_map is None:
            # If the drone has not published yet, do not block the mission.
            # Nav2/global/local costmaps remain the primary safety layer.
            return True

        return self.is_point_in_grid_safe(
            self.uav_aerial_map,
            x,
            y,
            clearance_radius,
            self.uav_blocked_threshold,
            self.uav_unknown_is_unsafe,
        )

    def is_point_combined_safe(
        self,
        x: float,
        y: float,
        nav_clearance_radius: Optional[float] = None,
        uav_clearance_radius: Optional[float] = None,
    ) -> bool:
        if nav_clearance_radius is None:
            nav_clearance_radius = self.candidate_clearance_radius

        if uav_clearance_radius is None:
            uav_clearance_radius = self.uav_candidate_clearance_radius

        nav_safe = self.is_point_costmap_safe(x, y, nav_clearance_radius)
        uav_safe = self.is_point_uav_aerial_safe(x, y, uav_clearance_radius)

        if not nav_safe:
            self.get_logger().warn(f"Point rejected by Nav2 global costmap: x={x:.2f}, y={y:.2f}")
            return False

        if not uav_safe:
            self.get_logger().warn(f"Point rejected by UAV aerial obstacle map: x={x:.2f}, y={y:.2f}")
            return False

        return True

    def is_physical_footprint_safe(
        self,
        grid: OccupancyGrid,
        x: float,
        y: float,
        yaw: float,
    ) -> bool:
        if grid is None or grid.info.resolution <= 0:
            return False
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            return False

        cost = self.footprint_max_cost(grid, x, y, yaw)

        if cost is None:
            return False

        return cost < self.costmap_lethal_threshold

    def is_rotation_sweep_safe(
        self,
        x: float,
        y: float,
        start_yaw: float,
        angular_z: float,
        duration_sec: float,
        step_sec: float = 0.05,
    ) -> bool:
        if self.global_costmap is None:
            return False

        if not all(
            math.isfinite(v)
            for v in (x, y, start_yaw, angular_z, duration_sec, step_sec)
        ):
            return False

        if duration_sec < 0.0 or step_sec <= 0.0:
            return False

        steps = max(1, math.ceil(duration_sec / step_sec))

        for i in range(steps + 1):
            t = min(i * step_sec, duration_sec)
            yaw = start_yaw + angular_z * t

            if not self.is_pose_inside_navigable_bounds(x, y):
                return False

            if not self.is_physical_footprint_safe(
                self.global_costmap,
                x,
                y,
                yaw,
            ):
                return False

        return True

    def is_point_costmap_safe(
        self,
        x: float,
        y: float,
        clearance_radius: Optional[float] = None,
    ) -> bool:
        if clearance_radius is None:
            clearance_radius = self.candidate_clearance_radius

        if self.global_costmap is None:
            self.get_logger().warn("Global costmap not received yet; candidate safety check is limited")
            return True

        info = self.global_costmap.info
        center = self.world_to_costmap_index(x, y)

        if center is None:
            return False

        cx, cy = center
        cells_radius = max(1, math.ceil(clearance_radius / info.resolution))

        for dy in range(-cells_radius, cells_radius + 1):
            for dx in range(-cells_radius, cells_radius + 1):
                if dx * dx + dy * dy > cells_radius * cells_radius:
                    continue

                mx = cx + dx
                my = cy + dy

                if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
                    return False

                value = self.costmap_cell_value(mx, my)

                if value == -1 and self.costmap_unknown_is_unsafe:
                    return False

                if value >= self.costmap_lethal_threshold:
                    return False

        return True

    def current_costmap_status(self) -> Tuple[bool, str]:
        """
        Validate the CURRENT ROBOT START POSE using both the centre point
        and the complete physical footprint.

        This must be consistent with path_is_valid().  A free centre cell
        does not mean the robot can safely start planning when part of its
        body overlaps a lethal/inflated obstacle region.
        """
        if self.global_costmap is None:
            return False, "global_costmap_unavailable"

        cell = self.world_to_costmap_index(
            self.current_x,
            self.current_y,
        )

        cell_value = None
        if cell is not None:
            cell_value = self.costmap_cell_value(*cell)

        point_safe = self.is_point_costmap_safe(
            self.current_x,
            self.current_y,
            clearance_radius=0.05,
        )

        footprint_cost = self.footprint_max_cost(
            self.global_costmap,
            self.current_x,
            self.current_y,
            self.current_yaw,
        )

        footprint_safe = (
            footprint_cost is not None
            and math.isfinite(float(footprint_cost))
            and float(footprint_cost) >= 0.0
            and float(footprint_cost) < 99.0
        )

        safe = bool(point_safe and footprint_safe)

        diagnostic = (
            f"cell={cell_value if cell_value is not None else 'unknown'}, "
            f"footprint={footprint_cost if footprint_cost is not None else 'unknown'}"
        )

        return safe, diagnostic

    def recover_from_blocked_start_pose(self, target_id: str) -> bool:
        start_safe, start_value = self.current_costmap_status()
        if start_safe:
            self.blocked_start_recovery_pose = None
            return False

        if (
            self.active_navigation_goal
            or self.navigation_cancel_pending
            or self.navigation_submission_uncertain
            or self.planner_submission_uncertain
        ):
            self.get_logger().error(
                f"Start pose is blocked for target_id={target_id}, but action "
                "transport is not IDLE; refusing manual recovery."
            )
            return False

        if self.start_pose_recovery_count >= self.max_start_pose_recoveries_per_cycle:
            self.get_logger().error(
                f"Start pose remains blocked for target_id={target_id}: "
                f"start=({self.current_x:.2f}, {self.current_y:.2f}), "
                f"cost={start_value}, recoveries="
                f"{self.start_pose_recovery_count}/"
                f"{self.max_start_pose_recoveries_per_cycle}. "
                "Mission target is preserved, but this cycle will fail."
            )
            return False

        previous = self.blocked_start_recovery_pose
        if previous is not None and math.hypot(self.current_x-previous[0],self.current_y-previous[1]) < 0.15:
            self.eval_emit("recovery_suppressed", reason="unchanged_blocked_start",
                           target_id=target_id, diagnostics=self.navigation_diagnostic_snapshot())
            return False
        self.blocked_start_recovery_pose = (self.current_x,self.current_y)
        self.start_pose_recovery_count += 1
        self.get_logger().warn(
            f"Start pose blocked by Nav2 costmap before candidate planning: "
            f"target_id={target_id}, start=({self.current_x:.2f}, "
            f"{self.current_y:.2f}), cost={start_value}. "
            "Executing fast recovery and preserving the current target."
        )

        if not self.escape_recovery(
            reason="start pose blocked by costmap",
            fast=True,
        ):
            return False

        # Do not clear costmaps here. Egress success must be proven by the
        # observed post-motion costmap, not by erasing the obstacle layer.
        return True

    def path_is_valid(self, path: Optional[Path]) -> bool:
        """Validate the COMPLETE execution path before accepting a candidate.

        Nav2 planner feasibility alone is not sufficient in the farm world:
        a mathematically feasible path may pass through an inflated / inscribed
        tree region that is unsafe for the real robot footprint.

        RC11 therefore performs an independent full-path safety validation
        against BOTH the Nav2 global costmap and the UAV aerial obstacle map.
        """
        if path is None or not hasattr(path, "poses") or len(path.poses) < 2:
            return False

        if path.header.frame_id != "map":
            self.get_logger().warn(
                f"Path rejected: unexpected frame '{path.header.frame_id}'"
            )
            return False

        if self.global_costmap is None:
            self.get_logger().warn(
                "Path rejected: global costmap unavailable for full-path safety validation"
            )
            return False

        # Bound computation while still checking the complete route densely.
        # At most ~120 footprint evaluations per candidate.
        step = max(1, len(path.poses) // 120)

        sampled = list(path.poses[::step])
        if sampled[-1] is not path.poses[-1]:
            sampled.append(path.poses[-1])

        for pose_stamped in sampled:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y

            if not self.is_pose_inside_navigable_bounds(x, y):
                self.get_logger().warn(
                    f"Path rejected: outside navigable bounds near "
                    f"x={x:.2f}, y={y:.2f}"
                )
                return False

            q = pose_stamped.pose.orientation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )

            # Evaluate the ROBOT FOOTPRINT, not only the path center point.
            footprint_cost = self.footprint_max_cost(
                self.global_costmap,
                x,
                y,
                yaw,
            )

            # Costmap convention in this stack:
            #   99  = inscribed / effectively blocked
            #   100 = lethal obstacle
            # Unknown/non-finite values are also unsafe for execution.
            if (
                footprint_cost is None
                or not math.isfinite(float(footprint_cost))
                or footprint_cost < 0
                or footprint_cost >= 99
            ):
                self.get_logger().warn(
                    f"Path rejected by full-path Nav2 safety check near "
                    f"x={x:.2f}, y={y:.2f}, footprint_cost={footprint_cost}"
                )
                return False

            # Preserve independent UAV-map safety validation.
            if not self.is_point_uav_aerial_safe(
                x,
                y,
                clearance_radius=self.uav_path_clearance_radius,
            ):
                self.get_logger().warn(
                    f"Path rejected by UAV aerial obstacle map near "
                    f"x={x:.2f}, y={y:.2f}"
                )
                return False

        return True

    def _mark_planner_submission_uncertain(self, send_future, label: str):
        """Own a late planner response until its action reaches TERMINAL state."""
        self.planner_submission_uncertain = True

        def _late_goal_response(done_future):
            try:
                handle = done_future.result()
            except Exception:
                self.planner_submission_uncertain = False
                return

            if handle is None or not handle.accepted:
                self.planner_submission_uncertain = False
                return

            try:
                result_future = handle.get_result_async()
            except Exception:
                # We cannot prove terminal ownership; keep the barrier asserted.
                return

            def _terminal(_):
                self.planner_submission_uncertain = False

            result_future.add_done_callback(_terminal)
            try:
                handle.cancel_goal_async()
            except Exception:
                # Terminal-result callback remains authoritative even if the
                # cancellation request itself could not be submitted.
                pass

        send_future.add_done_callback(_late_goal_response)
        self.get_logger().error(
            f"Planner goal-response timeout for {label}; late acceptance "
            "is owned until its terminal result before any new action starts"
        )

    def planner_failure_is_infrastructure(self, reason: str) -> bool:
        return reason in {
            "planner_server_unavailable",
            "planner_send_exception",
            "planner_goal_response_error",
            "planner_goal_rejected",
            "planner_empty_result",
            "planner_timeout",
            "planner_goal_response_timeout_recovered",
            "planner_goal_response_unresolved",
            "previous_planner_goal_unresolved",
            "planner_cancel_unconfirmed",
            "planner_result_error",
            "planner_transport_cleanup_timeout",
            "navigation_not_idle",
        }

    def _cancel_planner_goal_bounded(
        self,
        handle,
        result_future,
        label: str,
    ) -> bool:
        try:
            cancel_future = handle.cancel_goal_async()
        except Exception as exc:
            self.get_logger().error(
                f"Could not cancel planner goal for {label}: {exc}"
            )
            return False

        rclpy.spin_until_future_complete(
            self,
            cancel_future,
            timeout_sec=self.planner_cancel_timeout_s,
        )
        if not cancel_future.done() or cancel_future.exception() is not None:
            self.get_logger().error(
                f"Planner cancellation was not confirmed for {label}"
            )
            if result_future is not None:
                self.planner_submission_uncertain = True

                def _result_finished(_):
                    self.planner_submission_uncertain = False

                result_future.add_done_callback(_result_finished)
            return False

        deadline = time.monotonic() + self.planner_cancel_timeout_s
        while (
            rclpy.ok()
            and result_future is not None
            and not result_future.done()
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(self, timeout_sec=0.05)

        terminal = result_future is None or result_future.done()
        if not terminal:
            self.planner_submission_uncertain = True

            def _result_finished(_):
                self.planner_submission_uncertain = False

            result_future.add_done_callback(_result_finished)
        return terminal

    def wait_for_planner_transport_idle(
        self,
        timeout_sec: Optional[float] = None,
    ) -> bool:
        """
        Wait until every outstanding ComputePathToPose submission is fully
        resolved before allowing navigation.

        This is a transport/lifecycle barrier. It does not change candidate
        selection, path ranking, obstacle thresholds, or retry policy.
        """
        if not self.planner_submission_uncertain:
            return True

        wait_limit = (
            float(timeout_sec)
            if timeout_sec is not None
            else (
                self.planner_goal_response_timeout_s
                + self.planner_cancel_timeout_s
                + 1.0
            )
        )

        self.get_logger().warn(
            f"Planner transport is not idle; waiting up to "
            f"{wait_limit:.1f}s for late goal-response/cancellation cleanup "
            "before any NavigateToPose submission."
        )

        deadline = time.monotonic() + wait_limit

        while (
            rclpy.ok()
            and self.planner_submission_uncertain
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(self, timeout_sec=0.05)

        if self.planner_submission_uncertain:
            self.get_logger().error(
                "Planner transport did not return to IDLE within the bounded "
                "cleanup window. Navigation will not be submitted."
            )
            return False

        # Give executor/action callbacks one short bounded drain interval.
        self.spin_wall_duration(0.15)

        self.get_logger().info(
            "Planner transport returned to IDLE; navigation submission is safe."
        )
        return True

    def compute_path_to_pose_bounded(
        self,
        start_pose: PoseStamped,
        goal_pose: PoseStamped,
        *,
        timeout_sec: Optional[float] = None,
        label: str = "candidate path",
    ) -> Optional[Path]:
        """Compute one Nav2 path without touching NavigateToPose state."""
        self.last_planner_failure_reason = ""

        if self.active_navigation_goal or self.navigation_cancel_pending:
            self.last_planner_failure_reason = "navigation_not_idle"
            return None

        if self.planner_submission_uncertain:
            if not self.wait_for_planner_transport_idle(
                timeout_sec=self.planner_response_cleanup_timeout_s
            ):
                self.last_planner_failure_reason = (
                    "previous_planner_goal_unresolved"
                )
                return None

        client = self.compute_path_to_pose_client
        if not client.wait_for_server(
            timeout_sec=self.planner_server_wait_timeout_s
        ):
            self.last_planner_failure_reason = "planner_server_unavailable"
            return None

        request = ComputePathToPose.Goal()
        request.start = start_pose
        request.goal = goal_pose
        request.use_start = True

        result_timeout = (
            self.candidate_planning_timeout_s
            if timeout_sec is None
            else float(timeout_sec)
        )
        self.get_logger().info(
            f"Getting path (bounded <= {result_timeout:.1f}s)..."
        )

        try:
            send_future = client.send_goal_async(request)
        except Exception as exc:
            self.last_planner_failure_reason = "planner_send_exception"
            self.get_logger().warn(
                f"Planner send failed for {label}: {exc}"
            )
            return None

        rclpy.spin_until_future_complete(
            self,
            send_future,
            timeout_sec=self.planner_goal_response_timeout_s,
        )
        if not send_future.done():
            self._mark_planner_submission_uncertain(send_future, label)
            cleaned = self.wait_for_planner_transport_idle(
                timeout_sec=self.planner_response_cleanup_timeout_s
            )
            self.last_planner_failure_reason = (
                "planner_goal_response_timeout_recovered"
                if cleaned
                else "planner_goal_response_unresolved"
            )
            return None

        if send_future.exception() is not None:
            self.last_planner_failure_reason = "planner_goal_response_error"
            self.get_logger().warn(
                f"Planner goal-response failed for {label}: "
                f"{send_future.exception()}"
            )
            return None

        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.last_planner_failure_reason = "planner_goal_rejected"
            return None

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(
            self,
            result_future,
            timeout_sec=result_timeout,
        )

        if not result_future.done():
            canceled_terminal = self._cancel_planner_goal_bounded(
                handle,
                result_future,
                label,
            )
            self.last_planner_failure_reason = (
                "planner_timeout"
                if canceled_terminal
                else "planner_cancel_unconfirmed"
            )
            self.get_logger().warn(
                f"Planner result timed out for {label} after "
                f"{result_timeout:.1f}s"
            )
            return None

        if result_future.exception() is not None:
            self.last_planner_failure_reason = "planner_result_error"
            self.get_logger().warn(
                f"Planner result failed for {label}: "
                f"{result_future.exception()}"
            )
            return None

        response = result_future.result()
        if response is None:
            self.last_planner_failure_reason = "planner_empty_result"
            return None

        if response.status != GoalStatus.STATUS_SUCCEEDED:
            self.last_planner_failure_reason = (
                f"planner_status_{response.status}"
            )
            return None

        path = response.result.path
        if path is None or not hasattr(path, "poses") or len(path.poses) < 2:
            self.last_planner_failure_reason = "planner_empty_path"
            return None

        return path

    def get_feasible_path_to_pose(self, goal_pose: PoseStamped) -> Optional[Path]:
        if not self.is_pose_inside_navigable_bounds(
            goal_pose.pose.position.x,
            goal_pose.pose.position.y,
        ):
            self.last_planner_failure_reason = (
                "goal_outside_navigable_bounds"
            )
            return None

        start_pose = self.make_pose(
            self.current_x,
            self.current_y,
            self.current_yaw,
        )
        start_safe = self.is_point_costmap_safe(
            self.current_x,
            self.current_y,
            clearance_radius=0.05,
        )

        path = self.compute_path_to_pose_bounded(
            start_pose,
            goal_pose,
            timeout_sec=self.candidate_planning_timeout_s,
            label=(
                f"candidate goal "
                f"({goal_pose.pose.position.x:.2f},"
                f"{goal_pose.pose.position.y:.2f})"
            ),
        )

        if path is None:
            start_cell = self.world_to_costmap_index(
                self.current_x,
                self.current_y,
            )
            goal_cell = self.world_to_costmap_index(
                goal_pose.pose.position.x,
                goal_pose.pose.position.y,
            )
            start_value = "unknown"
            goal_value = "unknown"
            if self.global_costmap is not None and start_cell is not None:
                start_value = str(self.costmap_cell_value(*start_cell))
            if self.global_costmap is not None and goal_cell is not None:
                goal_value = str(self.costmap_cell_value(*goal_cell))

            self.get_logger().warn(
                f"Planner rejected path: reason="
                f"{self.last_planner_failure_reason or 'no_path'}, "
                f"start=({self.current_x:.2f},{self.current_y:.2f}) "
                f"safe={start_safe} cost={start_value}, "
                f"goal=({goal_pose.pose.position.x:.2f},"
                f"{goal_pose.pose.position.y:.2f}) cost={goal_value}"
            )
            return None

        if self.path_is_valid(path):
            self.last_planner_failure_reason = ""
            return path

        self.last_planner_failure_reason = (
            "path_infeasible_under_safety_checks"
        )
        return None

    def compute_path_length(self, path: Path) -> float:
        total = 0.0
        poses = path.poses
        for i in range(1, len(poses)):
            x1 = poses[i - 1].pose.position.x
            y1 = poses[i - 1].pose.position.y
            x2 = poses[i].pose.position.x
            y2 = poses[i].pose.position.y
            total += math.hypot(x2 - x1, y2 - y1)
        return total

    # ------------------------------------------------------------------
    # TARGET QUEUE
    # ------------------------------------------------------------------

    def make_target_id(self, x: float, y: float) -> str:
        return self.target_manager.make_target_id(x, y)

    def set_target_state(self, target: Dict, state: str):
        old_state, new_state = self.target_manager.set_target_state(target, state)
        self.get_logger().info(
            f"Target state transition: id={target['id']} "
            f"{old_state} -> {new_state}"
        )

    def is_duplicate_target(self, x: float, y: float) -> bool:
        return self.target_manager.is_duplicate_target(x, y)

    def add_target(self, x: float, y: float, source: str) -> bool:
        self.eval_target_order += 1
        incoming_order = self.eval_target_order
        added, target, reason = self.target_manager.add_target(x, y, source)
        target_id = self.make_target_id(x, y)
        self.eval_emit(
            "target_received",
            target_id=target_id,
            target_x=float(x),
            target_y=float(y),
            target_order=incoming_order,
            source=source,
            accepted=bool(added),
        )

        if reason == "invalid_coordinates":
            self.get_logger().error(
                f"Invalid target coordinates rejected from {source}: x={x}, y={y}"
            )
            return False

        if reason == "duplicate" and target is not None:
            self.get_logger().warn(
                f"Duplicate target ignored: id={target_id}, "
                f"state={target['state']}"
            )
            return False

        if not added:
            return False

        self.eval_target_orders[str(target["id"])] = incoming_order
        self.set_target_state(target, self.TARGET_QUEUED)
        self.eval_emit(
            "target_queued",
            target_id=target["id"],
            target_x=float(target["x"]),
            target_y=float(target["y"]),
            target_order=incoming_order,
            queue_size=len(self.target_queue),
            target_state=self.TARGET_QUEUED,
        )
        self.get_logger().info(
            f"Target queued: id={target_id}, source={source}, "
            f"x={x:.2f}, y={y:.2f}, pending={len(self.target_queue)}"
        )
        return True

    def scan_complete_callback(self, msg: UInt32):
        self.target_manager.mark_scan_complete(int(msg.data))
        self.get_logger().info(
            f"UAV scan-complete received: expected_targets={msg.data}, "
            f"registered_targets={len(self.targets_by_id)}"
        )

    def pose_target_callback(self, msg: PoseStamped):
        if msg.header.frame_id != "map":
            self.get_logger().error(
                f"Rejected UAV target. Expected frame_id='map', got '{msg.header.frame_id}'"
            )
            return
        self.add_target(msg.pose.position.x, msg.pose.position.y, "UAV /uav/tree_target")

    def clicked_point_callback(self, msg: PointStamped):
        if msg.header.frame_id != "map":
            self.get_logger().error(
                f"Rejected clicked point. Expected frame_id='map', got '{msg.header.frame_id}'"
            )
            return
        self.add_target(msg.point.x, msg.point.y, "RViz /clicked_point")

    def select_nearest_target(self) -> Optional[Dict[str, float]]:
        target = self.target_manager.select_nearest_target(self.distance_to)
        if target is None:
            return None

        self.activate_selected_target(target)
        return target

    def select_fifo_target(self) -> Optional[Dict[str, float]]:
        if self.current_target is not None:
            return self.current_target

        if not self.target_queue:
            return None

        target = self.target_queue.pop(0)
        self.current_target = target
        self.activate_selected_target(target)
        return target

    def select_target_for_method(self) -> Optional[Dict[str, float]]:
        if self.eval_method == "B3":
            return self.select_fifo_target()
        return self.select_nearest_target()

    def activate_selected_target(self, target: Dict[str, float]):
        if target is not self.current_target or target["state"] == self.TARGET_ACTIVE:
            return

        self.set_target_state(target, self.TARGET_ACTIVE)
        self.eval_active_target_id = str(target["id"])
        self.eval_emit(
            "target_selected",
            target_id=target["id"],
            target_x=float(target["x"]),
            target_y=float(target["y"]),
            target_order=self.eval_target_orders.get(str(target["id"]), self.eval_target_order),
            queue_size=len(self.target_queue),
            target_state=self.TARGET_ACTIVE,
            selection=True,
        )
        self.get_logger().info(
            f"Target selected: id={target['id']}, "
            f"pending={len(self.target_queue)}"
        )

    # ------------------------------------------------------------------
    # APPROACH CANDIDATES
    # ------------------------------------------------------------------

    def compute_candidate_approach_poses(self, target_x: float, target_y: float):
        angles_deg = [0, 30, 60, 90, 120, 150, 180, 210, 240, 270, 300, 330]
        candidates = []

        for radius in self.service_rings:
            for angle_deg in angles_deg:
                angle = math.radians(angle_deg)
                approach_x = target_x + radius * math.cos(angle)
                approach_y = target_y + radius * math.sin(angle)

                yaw = math.atan2(target_y - approach_y, target_x - approach_x)
                distance_from_robot = self.distance_to(approach_x, approach_y)

                nav_safe = self.is_point_costmap_safe(
                    approach_x,
                    approach_y,
                    clearance_radius=self.candidate_clearance_radius,
                )
                uav_safe = self.is_point_uav_aerial_safe(
                    approach_x,
                    approach_y,
                    clearance_radius=self.uav_candidate_clearance_radius,
                )

                candidates.append(
                    {
                        "x": approach_x,
                        "y": approach_y,
                        "yaw": yaw,
                        "distance": distance_from_robot,
                        "angle_deg": angle_deg,
                        "radius": radius,
                        "candidate_id": f"r{radius:.2f}_a{angle_deg:03d}",
                        "nav_precheck_safe": nav_safe,
                        "uav_safe": uav_safe,
                    }
                )

        # Check nearby safe candidates first. Nav2 path length remains the
        # final selection criterion once the bounded prepared set is ready.
        candidates.sort(
            key=lambda c: (
                not c["uav_safe"],
                not c["nav_precheck_safe"],
                c["distance"],
                c["radius"],
            )
        )
        return candidates[: self.max_candidates_checked]

    def compute_single_candidate_approach_pose(self, target_x: float, target_y: float):
        radius = self.single_candidate_radius_m
        angle_deg = self.single_candidate_angle_deg
        angle = math.radians(angle_deg)
        approach_x = target_x + radius * math.cos(angle)
        approach_y = target_y + radius * math.sin(angle)
        yaw = math.atan2(target_y - approach_y, target_x - approach_x)

        nav_safe = self.is_point_costmap_safe(
            approach_x,
            approach_y,
            clearance_radius=self.candidate_clearance_radius,
        )
        uav_safe = self.is_point_uav_aerial_safe(
            approach_x,
            approach_y,
            clearance_radius=self.uav_candidate_clearance_radius,
        )

        return {
            "x": approach_x,
            "y": approach_y,
            "yaw": yaw,
            "angle_deg": angle_deg,
            "radius": radius,
            "candidate_id": f"single_r{radius:.2f}_a{angle_deg:03d}",
            "nav_precheck_safe": nav_safe,
            "uav_safe": uav_safe,
        }

    # ------------------------------------------------------------------
    # NAVIGATION
    # ------------------------------------------------------------------

    def target_is_treated(self, target_x: float, target_y: float) -> bool:
        return self.distance_to(target_x, target_y) <= self.treatment_reach_radius

    def goal_success_check(
        self,
        service_x: float,
        service_y: float,
        label: str,
        target_center: Optional[Tuple[float, float]] = None,
        service_acceptance_radius: Optional[float] = None,
    ) -> bool:
        if service_acceptance_radius is None:
            service_acceptance_radius = self.service_acceptance_radius

        dist_to_service = self.distance_to(service_x, service_y)

        if target_center is not None:
            tx, ty = target_center
            dist_to_target = self.distance_to(tx, ty)

            # Mission success is defined by physical treatment reach.
            # The selected service pose is only an approach waypoint; once the
            # robot is safely within treatment range of the target, continuing
            # toward that waypoint can force unnecessary motion into vegetation
            # or tree obstacles.
            if dist_to_target <= self.treatment_reach_radius:
                self.get_logger().info(
                    f"Treatment reach achieved: target distance "
                    f"{dist_to_target:.2f} m <= {self.treatment_reach_radius:.2f} m "
                    f"(service distance={dist_to_service:.2f} m). "
                    "Stopping navigation and treating from the current safe pose."
                )
                return True

            if dist_to_service <= service_acceptance_radius:
                self.get_logger().warn(
                    f"Reached service point but target is still too far: "
                    f"target distance={dist_to_target:.2f} m > {self.treatment_reach_radius:.2f} m. "
                    "Keeping the SAME target and replanning closer."
                )
                return False

            return False

        # Home / generic goal success.
        if dist_to_service <= service_acceptance_radius:
            self.get_logger().info(
                f"Close enough to {label}: distance "
                f"{dist_to_service:.2f} m <= {service_acceptance_radius:.2f} m"
            )
            return True

        return False

    def eval_emit_target_reached(self, target_id: Optional[str], attempt: int, goal_x: float, goal_y: float, target_center=None):
        target_dist = None
        if target_center is not None:
            target_dist = self.distance_to(target_center[0], target_center[1])
        service_dist = self.distance_to(goal_x, goal_y)
        nav_time = None
        if self.eval_nav_started_sim is not None:
            nav_time = self.eval_sim_time() - self.eval_nav_started_sim
        self.eval_emit(
            "target_reached",
            target_id=target_id, attempt=attempt,
            candidate_id=self.eval_active_candidate_id,
            target_x=(target_center[0] if target_center is not None else goal_x),
            target_y=(target_center[1] if target_center is not None else goal_y),
            robot_x=float(self.current_x), robot_y=float(self.current_y),
            final_target_distance_m=target_dist, final_service_distance_m=service_dist,
            navigation_time_s=nav_time, actual_path_length_m=self.eval_actual_path_length,
        )

    def _mark_navigation_submission_uncertain(self, send_future, label: str):
        """Own a late NavigateToPose response through terminal result."""
        self.navigation_submission_uncertain = True

        def _late_nav_response(done_future):
            try:
                handle = done_future.result()
            except Exception:
                self.navigation_submission_uncertain = False
                return

            if handle is None or not handle.accepted:
                self.navigation_submission_uncertain = False
                return

            self.goal_handle = handle
            self.active_navigation_goal = True
            self.navigation_cancel_pending = True
            try:
                result_future = handle.get_result_async()
                self.result_future = result_future
            except Exception:
                return

            def _terminal(done_result):
                try:
                    response = done_result.result()
                    if response is not None:
                        self.status = response.status
                except Exception:
                    pass
                self.active_navigation_goal = False
                self.navigation_cancel_pending = False
                self.navigation_submission_uncertain = False

            result_future.add_done_callback(_terminal)
            try:
                handle.cancel_goal_async()
            except Exception:
                pass

        send_future.add_done_callback(_late_nav_response)
        self.get_logger().error(
            f"NavigateToPose goal-response timeout for {label}; any late "
            "acceptance is owned and canceled through terminal result"
        )

    def wait_for_navigation_transport_idle(
        self,
        label: str = "navigation transport",
        timeout_sec: Optional[float] = None,
    ) -> bool:
        wait_limit = (
            self.nav_response_cleanup_timeout_s
            if timeout_sec is None
            else float(timeout_sec)
        )
        deadline = time.monotonic() + max(0.0, wait_limit)
        while (
            rclpy.ok()
            and self.navigation_submission_uncertain
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.navigation_submission_uncertain:
            self.get_logger().error(
                f"{label} did not return to terminal/IDLE state within "
                f"{wait_limit:.1f}s"
            )
            return False
        return not self.active_navigation_goal and not self.navigation_cancel_pending

    def navigation_failure_is_infrastructure(self, reason: str) -> bool:
        return reason in {
            "nav2_action_server_unavailable",
            "nav2_goal_send_exception",
            "nav2_goal_response_error",
            "nav2_goal_rejected",
            "nav2_goal_response_timeout_recovered",
            "nav2_goal_response_timeout_unresolved",
            "planner_transport_not_idle_before_navigation",
            "previous_nav_goal_submission_unresolved",
        }

    def drain_navigation_before_shutdown(self, reason: str) -> bool:
        """Bounded final planner/navigation ownership cleanup before shutdown."""
        ok = True
        if self.planner_submission_uncertain:
            ok = self.wait_for_planner_transport_idle(
                timeout_sec=self.planner_response_cleanup_timeout_s
            ) and ok
        if self.active_navigation_goal and not self.navigation_cancel_pending:
            ok = self.cancel_active_navigation_and_wait(reason) and ok
        if self.navigation_submission_uncertain:
            ok = self.wait_for_navigation_transport_idle(
                reason, timeout_sec=self.nav_response_cleanup_timeout_s
            ) and ok
        if self.active_navigation_goal or self.navigation_cancel_pending:
            ok = self.wait_until_nav2_idle(
                reason, timeout_sec=self.nav_response_cleanup_timeout_s
            ) and ok
        return ok

    def submit_navigation_goal_bounded(
        self,
        pose: PoseStamped,
        label: str,
    ) -> bool:
        """Submit NavigateToPose with bounded server/goal-response ownership."""
        if not self.wait_for_planner_transport_idle(
            timeout_sec=self.planner_response_cleanup_timeout_s
        ):
            self.last_navigation_reason = "planner_transport_not_idle_before_navigation"
            return False

        if self.navigation_submission_uncertain:
            if not self.wait_for_navigation_transport_idle(
                "previous NavigateToPose submission",
                timeout_sec=self.nav_response_cleanup_timeout_s,
            ):
                self.last_navigation_reason = "previous_nav_goal_submission_unresolved"
                return False

        client = self.nav_to_pose_client
        if not client.wait_for_server(timeout_sec=self.nav_action_server_wait_timeout_s):
            self.last_navigation_reason = "nav2_action_server_unavailable"
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_msg.behavior_tree = ""
        self.feedback = None
        self.status = None

        try:
            send_future = client.send_goal_async(goal_msg, self._feedbackCallback)
        except Exception as exc:
            self.last_navigation_reason = "nav2_goal_send_exception"
            self.get_logger().error(f"NavigateToPose send failed for {label}: {exc}")
            return False

        rclpy.spin_until_future_complete(
            self, send_future, timeout_sec=self.nav_goal_response_timeout_s
        )
        if not send_future.done():
            self._mark_navigation_submission_uncertain(send_future, label)
            cleaned = self.wait_for_navigation_transport_idle(
                f"late NavigateToPose response for {label}",
                timeout_sec=self.nav_response_cleanup_timeout_s,
            )
            self.last_navigation_reason = (
                "nav2_goal_response_timeout_recovered"
                if cleaned
                else "nav2_goal_response_timeout_unresolved"
            )
            if not cleaned:
                self.halt_mission_for_navigation_fault(
                    f"NavigateToPose late response unresolved for {label}"
                )
            return False

        if send_future.exception() is not None:
            self.last_navigation_reason = "nav2_goal_response_error"
            self.get_logger().error(
                f"NavigateToPose goal-response failed for {label}: "
                f"{send_future.exception()}"
            )
            return False

        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.last_navigation_reason = "nav2_goal_rejected"
            return False

        self.goal_handle = handle
        self.result_future = handle.get_result_async()
        return True

    def go_to_pose_and_wait(
        self,
        pose: PoseStamped,
        label: str,
        acceptance_radius: Optional[float] = None,
        timeout_sec: float = 120.0,
        target_id: Optional[str] = "unknown",
        candidate_index: Optional[int] = None,
        attempt: int = 0,
        target_center: Optional[Tuple[float, float]] = None,
        allow_front_obstacle_replan: bool = True,
        enable_stuck_recovery: bool = True,
    ) -> bool:
        if acceptance_radius is None:
            acceptance_radius = self.service_acceptance_radius

        goal_x = pose.pose.position.x
        goal_y = pose.pose.position.y
        self.last_navigation_result = "none"
        self.last_navigation_reason = "not_started"
        self.last_navigation_terminal_status = "not_started"
        self.last_navigation_terminal_result = "not_started"
        self.last_navigation_terminal_reason = "not_started"

        if pose.header.frame_id != "map" or not self.is_pose_inside_navigable_bounds(goal_x, goal_y):
            self.last_navigation_reason = "map_boundary"
            self.eval_emit("goal_rejected", reason="map_boundary", goal_x=goal_x, goal_y=goal_y)
            return False

        candidate_text = (
            str(candidate_index) if candidate_index is not None else "none"
        )

        self.get_logger().info(
            f"Navigating to {label}: target_id={target_id}, "
            f"candidate={candidate_text}, x={goal_x:.2f}, y={goal_y:.2f}, "
            f"timeout={timeout_sec:.1f}s"
        )

        if not rclpy.ok():
            return False

        if self.active_navigation_goal or self.navigation_cancel_pending:
            self.get_logger().warn(
                f"attempted to submit target_id={target_id}, "
                f"candidate={candidate_text} while NavigateToPose is still "
                "active/canceling; keeping the same target ACTIVE"
            )
            return False

        if not self.wait_until_nav2_idle(
            f"before submitting target_id={target_id}, candidate={candidate_text}",
            timeout_sec=0.0,
        ):
            self.get_logger().warn(
                f"NavigateToPose is not idle before submitting target_id={target_id}, "
                f"candidate={candidate_text}; keeping the same target ACTIVE"
            )
            return False

        # A rejected BasicNavigator goal does not replace result_future.
        # Clear completed task references before submission so stale state
        # can never be inspected for the new candidate.
        self.goal_handle = None
        self.result_future = None
        self.status = None

        cooldown_until = self.last_navigation_submission_time + self.min_replan_interval_s
        if self.physical_now() < cooldown_until:
            self.eval_emit("replan_suppressed", reason="submission_cooldown",
                           wait_s=cooldown_until-self.physical_now())
            while rclpy.ok() and self.physical_now() < cooldown_until:
                rclpy.spin_once(self, timeout_sec=0.05)
        if not rclpy.ok():
            return False
        if not self.wait_for_valid_localization():
            self.last_navigation_reason = self.localization_reason
            return False
        self.last_navigation_submission_time = self.physical_now()
        accepted = self.submit_navigation_goal_bounded(pose, label)
        if not accepted:
            self.last_navigation_terminal_status = "rejected"
            self.last_navigation_terminal_result = "rejected"
            self.last_navigation_terminal_reason = (
                self.last_navigation_reason or "nav2_goal_rejected"
            )
            self.last_navigation_result = "rejected"
            if self.last_navigation_reason == "not_started":
                self.last_navigation_reason = "nav2_goal_rejected"
            self.eval_emit(
                "goal_rejected",
                target_id=target_id, attempt=attempt, candidate_id=self.eval_active_candidate_id,
                goal_x=goal_x, goal_y=goal_y,
                rejection_reason=self.last_navigation_reason or "nav2_goal_rejected",
            )
            self.goal_handle = None
            self.result_future = None
            self.get_logger().error(
                f"NavigateToPose goal rejected: target_id={target_id}, "
                f"candidate={candidate_text}, label={label}"
            )
            return False

        self.active_navigation_goal = True
        self.eval_active_target_id = target_id
        self.eval_target_attempt = attempt
        self.eval_active_candidate_index = candidate_index
        self.eval_active_goal_x = goal_x
        self.eval_active_goal_y = goal_y
        self.eval_nav_started_monotonic = time.monotonic()
        self.eval_nav_started_sim = self.eval_sim_time()
        self.eval_last_progress_monotonic = self.eval_nav_started_monotonic
        self.eval_last_progress_x = self.current_x
        self.eval_last_progress_y = self.current_y
        self.eval_actual_path_length = 0.0
        self.eval_emit(
            "goal_accepted",
            target_id=target_id, attempt=attempt,
            candidate_id=self.eval_active_candidate_id,
            goal_x=goal_x, goal_y=goal_y,
        )
        self.manual_recovery_active = False
        self.stop_command_latched = False
        self.motion_state = "NAVIGATING"
        self.get_logger().info(
            f"NavigateToPose goal accepted: target_id={target_id}, "
            f"candidate={candidate_text}, label={label}"
        )
        self.get_logger().info("Entering NAVIGATING; Nav2 owns continuous velocity")

        start_time = self.physical_now()
        timeout_extensions = 0
        max_timeout_extensions = 2
        service_checkpoint_distance = self.distance_to(goal_x, goal_y)
        initial_service_distance = service_checkpoint_distance
        service_checkpoint_time = self.physical_now()
        euclidean_progress_monitor_enabled = (
            initial_service_distance <= self.target_focus_radius
        )

        if target_center is not None:
            initial_target_distance = self.distance_to(target_center[0], target_center[1])
            target_checkpoint_distance = initial_target_distance
            target_checkpoint_time = self.physical_now()
            if initial_target_distance <= self.target_focus_radius:
                euclidean_progress_monitor_enabled = True
        else:
            initial_target_distance = None
            target_checkpoint_distance = None
            target_checkpoint_time = self.physical_now()

        if not euclidean_progress_monitor_enabled:
            self.get_logger().info(
                f"Long-route navigation for {label}: direct-distance stall "
                "monitoring is disabled so Nav2 can follow detours around "
                "obstacles"
            )

        obstacle_since = None
        progress_xy = (self.current_x, self.current_y)
        progress_time = self.physical_now()
        remaining_checkpoint = None
        while rclpy.ok() and not self.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.05)
            valid_localization = self.drain_localization_tf_callbacks()
            if not valid_localization and self.localization_reason in ("tf_missing", "tf_stale"):
                valid_localization = self.drain_localization_tf_callbacks()
            if not valid_localization or not self.is_pose_inside_navigable_bounds(self.current_x, self.current_y):
                self.last_navigation_reason = self.localization_reason if not valid_localization else "map_boundary"
                self.cancel_active_navigation_and_wait(self.last_navigation_reason)
                return False

            now = self.physical_now()
            feedback = self.getFeedback()
            remaining = getattr(feedback, "distance_remaining", None)
            moved = math.hypot(self.current_x-progress_xy[0], self.current_y-progress_xy[1]) >= 0.15
            path_progress = (remaining is not None and remaining_checkpoint is not None
                             and remaining_checkpoint-remaining >= 0.15)
            if moved or path_progress:
                progress_time = now
                progress_xy = (self.current_x, self.current_y)
                remaining_checkpoint = remaining
                service_checkpoint_time = target_checkpoint_time = now
            elif remaining_checkpoint is None:
                remaining_checkpoint = remaining
            if not self.front_obstacle_detected and obstacle_since is not None:
                self.eval_emit("dynamic_obstacle_cleared", observation_s=now-obstacle_since)
                obstacle_since = None

            service_dist = self.distance_to(goal_x, goal_y)

            if target_center is not None:
                tx, ty = target_center
                target_dist = self.distance_to(tx, ty)
                self.get_logger().info(
                    f"Distance to {label}: service={service_dist:.2f} m, target={target_dist:.2f} m"
                )
            else:
                target_dist = None
                self.get_logger().info(f"Distance to {label}: {service_dist:.2f} m")

            # Structured progress logging (~2 Hz) for actual path length and timing.
            now_monotonic = time.monotonic()
            if self.eval_last_progress_x is not None and self.eval_last_progress_y is not None:
                self.eval_actual_path_length += math.hypot(
                    self.current_x - self.eval_last_progress_x,
                    self.current_y - self.eval_last_progress_y,
                )
            self.eval_last_progress_x = self.current_x
            self.eval_last_progress_y = self.current_y
            self.eval_last_progress_monotonic = now_monotonic
            self.eval_emit(
                "navigation_progress",
                target_id=target_id, attempt=attempt,
                candidate_id=self.eval_active_candidate_id,
                robot_x=float(self.current_x), robot_y=float(self.current_y),
                target_distance_m=target_dist, service_distance_m=service_dist,
                linear_velocity_mps=None,
                progress_target_m=(initial_target_distance - target_dist) if target_dist is not None and initial_target_distance is not None else None,
                progress_service_m=initial_service_distance - service_dist,
            )

            # Main success condition for crop/tree/home.
            if self.goal_success_check(
                goal_x,
                goal_y,
                label,
                target_center=target_center,
                service_acceptance_radius=acceptance_radius,
            ):
                if not self.cancel_active_navigation_and_wait(
                    f"{label} success condition"
                ):
                    return False

                # RC12 invariant:
                # Navigation may move slightly while a successful-reach cancel
                # is reaching its terminal state. Revalidate the ACTUAL target
                # distance after cancellation before declaring target_reached.
                if target_center is not None:
                    tx, ty = target_center
                    post_cancel_target_distance = self.distance_to(tx, ty)

                    if post_cancel_target_distance > self.treatment_reach_radius:
                        self.last_navigation_result = (
                            "post_cancel_outside_treatment_radius"
                        )
                        self.last_navigation_reason = (
                            "post_cancel_outside_treatment_radius"
                        )
                        self.get_logger().warn(
                            f"Post-cancel treatment reach revalidation failed: "
                            f"target distance={post_cancel_target_distance:.6f} m > "
                            f"{self.treatment_reach_radius:.6f} m. "
                            "Treatment will NOT start; the target remains unsuccessful "
                            "for the current navigation attempt."
                        )
                        self.stop_robot()
                        return False

                self.eval_emit_target_reached(
                    target_id,
                    attempt,
                    goal_x,
                    goal_y,
                    target_center,
                )
                self.stop_robot()
                return True

            # If we are near the active target, do not let Nav2 take a strange far detour.
            if target_center is not None and initial_target_distance is not None:
                if initial_target_distance <= self.target_focus_radius:
                    if target_dist is not None and target_dist > initial_target_distance + self.max_allowed_retreat_from_target:
                        self.eval_emit(
                            "stall_detected", target_id=target_id, attempt=attempt,
                            candidate_id=self.eval_active_candidate_id, stall_type="target_retreat",
                            window_s=0.0, progress_m=initial_target_distance - target_dist,
                            threshold_m=-self.max_allowed_retreat_from_target, remaining_distance_m=target_dist,
                        )
                        self.get_logger().warn(
                            f"Robot is moving away from active target: target distance "
                            f"{target_dist:.2f} m > initial {initial_target_distance:.2f} m + "
                            f"{self.max_allowed_retreat_from_target:.2f} m. Replanning SAME target."
                        )
                        if not self.cancel_active_navigation_and_wait(
                            f"{label} retreat detection"
                        ):
                            return False

                        self.post_nav2_cancel_recovery(
                            "no useful progress / stuck",
                            fast=False,
                        )
                        return False

                if (
                    euclidean_progress_monitor_enabled
                    and
                    target_dist is not None
                    and target_checkpoint_distance is not None
                    and target_checkpoint_distance - target_dist
                    >= self.min_target_progress_required
                ):
                    progress = target_checkpoint_distance - target_dist
                    target_checkpoint_distance = target_dist
                    target_checkpoint_time = self.physical_now()
                    self.get_logger().info(
                        f"Target progress checkpoint reset for {label}: "
                        f"progress={progress:.2f} m, remaining={target_dist:.2f} m"
                    )

                if (
                    euclidean_progress_monitor_enabled
                    and
                    target_dist is not None
                    and self.physical_now() - target_checkpoint_time
                    > self.target_progress_window_sec
                ):
                    progress = (
                        target_checkpoint_distance - target_dist
                        if target_checkpoint_distance is not None
                        else 0.0
                    )
                    self.get_logger().warn(
                        f"Rolling target stall detected for {label}: "
                        f"progress={progress:.2f} m < "
                        f"{self.min_target_progress_required:.2f} m during "
                        f"{self.target_progress_window_sec:.1f}s, "
                        f"remaining={target_dist:.2f} m"
                    )
                    if not self.cancel_active_navigation_and_wait(
                        f"{label} target stall"
                    ):
                        return False

                    self.post_nav2_cancel_recovery(
                        "stuck or blocked path",
                        fast=False,
                    )
                    return False

            # Observe while Nav2 remains the sole velocity authority. Laser proximity
            # is evidence of blockage, not proof of object motion or a collision.
            if allow_front_obstacle_replan and self.front_obstacle_detected:
                near_target = target_center is not None and self.distance_to(*target_center) <= self.treatment_reach_radius + 0.20
                if not near_target:
                    if obstacle_since is None:
                        obstacle_since = now
                        self.eval_emit("dynamic_obstacle_wait", reason="front_blockage_observation")
                    if (now-obstacle_since >= self.dynamic_obstacle_observation_s
                            and now-progress_time >= self.dynamic_obstacle_observation_s
                            and now-self.last_obstacle_replan_time >= self.min_replan_interval_s):
                        self.eval_emit("dynamic_obstacle_persistent", observation_s=now-obstacle_since)
                        if not self.cancel_active_navigation_and_wait(
                                f"{label} persistent front obstacle",
                                timeout_sec=self.dynamic_obstacle_cancel_timeout_sec):
                            return False
                        self.last_obstacle_replan_time = self.physical_now()
                        # First escalation is a local clear and replan. Physical escape
                        # remains available for separately detected blocked starts/stalls.
                        self.clear_local_costmap_bounded()
                        return False

            # Service-goal stuck recovery.
            if enable_stuck_recovery and euclidean_progress_monitor_enabled:
                if (
                    service_checkpoint_distance - service_dist
                    >= self.min_progress_required
                ):
                    progress = service_checkpoint_distance - service_dist
                    service_checkpoint_distance = service_dist
                    service_checkpoint_time = self.physical_now()
                    self.get_logger().info(
                        f"Service progress checkpoint reset for {label}: "
                        f"progress={progress:.2f} m, remaining={service_dist:.2f} m"
                    )

                if (
                    self.physical_now() - service_checkpoint_time
                    > self.stuck_check_window_sec
                ):
                    progress = service_checkpoint_distance - service_dist

                    if self.goal_success_check(
                        goal_x,
                        goal_y,
                        label,
                        target_center=target_center,
                        service_acceptance_radius=acceptance_radius,
                    ):
                        if not self.cancel_active_navigation_and_wait(
                            f"{label} service success condition"
                        ):
                            return False

                        self.stop_robot()
                        return True

                    self.eval_emit(
                        "stall_detected", target_id=target_id, attempt=attempt,
                        candidate_id=self.eval_active_candidate_id, stall_type="service_stall",
                        window_s=self.stuck_check_window_sec, progress_m=progress,
                        threshold_m=self.min_progress_required, remaining_distance_m=service_dist,
                    )
                    self.get_logger().warn(
                        f"Rolling service stall detected for {label}: "
                        f"progress={progress:.2f} m < "
                        f"{self.min_progress_required:.2f} m during "
                        f"{self.stuck_check_window_sec:.1f}s, "
                        f"remaining={service_dist:.2f} m"
                    )
                    if not self.cancel_active_navigation_and_wait(
                        f"{label} service stall"
                    ):
                        return False

                    self.post_nav2_cancel_recovery(
                        "stuck or blocked path",
                        fast=False,
                    )
                    return False

            if self.physical_now() - start_time > timeout_sec:
                if self.goal_success_check(
                    goal_x,
                    goal_y,
                    label,
                    target_center=target_center,
                    service_acceptance_radius=acceptance_radius,
                ):
                    if not self.cancel_active_navigation_and_wait(
                        f"{label} timeout success condition"
                    ):
                        return False

                    self.stop_robot()
                    self.last_navigation_result = "succeeded"
                    self.last_navigation_reason = "mission_success_condition"
                    return True

                now = self.physical_now()
                recent_service_progress = (
                    enable_stuck_recovery
                    and now - service_checkpoint_time
                    <= self.stuck_check_window_sec
                )
                recent_target_progress = (
                    target_center is not None
                    and now - target_checkpoint_time
                    <= self.target_progress_window_sec
                )
                if (
                    timeout_extensions < max_timeout_extensions
                    and (recent_service_progress or recent_target_progress)
                ):
                    timeout_extensions += 1
                    start_time = now
                    self.get_logger().warn(
                        f"Extending navigation timeout for {label}: "
                        f"extension={timeout_extensions}/"
                        f"{max_timeout_extensions}, service={service_dist:.2f} m, "
                        f"recent_service_progress={recent_service_progress}, "
                        f"recent_target_progress={recent_target_progress}"
                    )
                    continue

                self.get_logger().warn(f"Timeout while navigating to {label}. Replanning SAME target.")
                if not self.cancel_active_navigation_and_wait(
                    f"{label} navigation timeout"
                ):
                    return False

                self.last_navigation_result = "timeout"
                self.last_navigation_reason = "navigation_timeout"
                if allow_front_obstacle_replan or enable_stuck_recovery:
                    self.post_nav2_cancel_recovery(
                        "navigation timeout",
                        fast=False,
                    )
                else:
                    self.stop_robot()
                return False

            self.spin_for_duration(0.45)

        # Capture one final pose sample before consuming the terminal result.
        if self.eval_last_progress_x is not None and self.eval_last_progress_y is not None:
            self.eval_actual_path_length += math.hypot(
                self.current_x - self.eval_last_progress_x,
                self.current_y - self.eval_last_progress_y,
            )
            self.eval_emit(
                "navigation_progress",
                target_id=target_id, attempt=attempt,
                candidate_id=self.eval_active_candidate_id,
                robot_x=float(self.current_x), robot_y=float(self.current_y),
                target_distance_m=(self.distance_to(target_center[0], target_center[1]) if target_center is not None else None),
                service_distance_m=self.distance_to(goal_x, goal_y),
                linear_velocity_mps=None, progress_target_m=None, progress_service_m=None,
            )

        self.active_navigation_goal = False
        self.navigation_cancel_pending = False
        self.motion_state = "STOPPED"
        result = self.getResult()
        self.capture_navigation_terminal(result, "nav2_terminal")
        self.goal_handle = None
        self.result_future = None
        self.status = None

        if result == TaskResult.SUCCEEDED:
            if target_center is not None:
                tx, ty = target_center
                if not self.target_is_treated(tx, ty):
                    self.last_navigation_result = "succeeded"
                    self.last_navigation_reason = "service_pose_reached_target_outside_treatment_radius"
                    self.get_logger().warn(
                        "Nav2 reached service pose, but target is not within treatment radius. "
                        "Replanning SAME target closer."
                    )
                    self.stop_robot()
                    return False

            self.get_logger().info(f"Arrived at {label}")
            self.eval_emit_target_reached(target_id, attempt, goal_x, goal_y, target_center)
            self.stop_robot()
            self.last_navigation_result = "succeeded"
            self.last_navigation_reason = "nav2_succeeded"
            return True

        if self.goal_success_check(
            goal_x,
            goal_y,
            label,
            target_center=target_center,
            service_acceptance_radius=acceptance_radius,
        ):
            self.get_logger().warn("Nav2 returned non-success, but treatment/home condition is satisfied.")
            self.eval_emit_target_reached(target_id, attempt, goal_x, goal_y, target_center)
            self.stop_robot()
            self.last_navigation_result = "non_success_but_within_acceptance"
            self.last_navigation_reason = "mission_success_condition"
            return True

        if result == TaskResult.CANCELED:
            self.last_navigation_result = "canceled"
            self.last_navigation_reason = "nav2_canceled"
            self.get_logger().warn(f"Navigation to {label} was canceled; will replan SAME target")
        elif result == TaskResult.FAILED:
            self.last_navigation_result = "failed"
            self.last_navigation_reason = "nav2_failed"
            self.get_logger().warn(f"Nav2 failed to reach {label}; will replan SAME target")
        else:
            self.last_navigation_result = "unknown"
            self.last_navigation_reason = f"nav2_result_{result}"
            self.get_logger().warn(f"Unknown navigation result for {label}; will replan SAME target")

        self.stop_robot()
        if allow_front_obstacle_replan or enable_stuck_recovery:
            self.clear_costmaps_safe()
        return False

    def plan_feasible_candidates(
        self,
        target_x: float,
        target_y: float,
        failed_candidate_keys: Optional[Set[Tuple[float, int]]] = None,
        target_id: Optional[str] = None,
        attempt: int = 0,
    ):
        failed_candidate_keys = failed_candidate_keys or set()
        target_id = target_id or f"target@({target_x:.2f},{target_y:.2f})"
        self.last_candidate_batch_infrastructure_reason = ""
        start_safe, start_value = self.current_costmap_status()

        if not start_safe:
            self.get_logger().warn(
                f"Blocked robot footprint detected BEFORE candidate planning: "
                f"target_id={target_id}, pose=({self.current_x:.2f}, "
                f"{self.current_y:.2f}), {start_value}. Executing bounded "
                "reverse egress before the 32-candidate batch."
            )
            self.blocked_start_recovery_pose = None
            for recovery_index in range(self.max_start_pose_recoveries_per_cycle):
                self.start_pose_recovery_count = recovery_index
                if not self.recover_from_blocked_start_pose(target_id):
                    break
                self.spin_wall_duration(0.50)
                start_safe, start_value = self.current_costmap_status()
                if start_safe:
                    self.get_logger().info(
                        f"Blocked-start recovery succeeded for target_id={target_id}: "
                        f"new_pose=({self.current_x:.2f}, {self.current_y:.2f}), "
                        f"{start_value}. Starting candidate planning."
                    )
                    self.blocked_start_recovery_pose = None
                    break
                # Permit the next bounded egress step from the newly measured pose.
                self.blocked_start_recovery_pose = None

            if not start_safe:
                reason = f"start footprint remains blocked after bounded egress; {start_value}"
                self.current_candidate_rejections.append(reason)
                self.get_logger().warn(reason)
                return []

        candidates = self.compute_candidate_approach_poses(target_x, target_y)
        feasible_candidates = []

        for i, candidate in enumerate(candidates, start=1):
            if not rclpy.ok():
                return []

            self.get_logger().info(
                f"Checking candidate {i}/{len(candidates)} "
                f"angle={candidate['angle_deg']} deg radius={candidate['radius']:.2f} "
                f"x={candidate['x']:.2f}, y={candidate['y']:.2f}, "
                f"nav_precheck_safe={candidate['nav_precheck_safe']}, "
                f"uav_safe={candidate['uav_safe']}"
            )

            candidate_key = (
                round(candidate["radius"], 3),
                int(candidate["angle_deg"]),
            )
            candidate_id = candidate["candidate_id"]
            rejected_reason = ""
            planner_feasible = False
            path_len = None
            planning_start = time.monotonic()

            if not self.is_pose_inside_navigable_bounds(candidate["x"], candidate["y"]):
                self.eval_emit("candidate_checked", target_id=target_id, attempt=attempt,
                               candidate_id=candidate_id, x=candidate["x"], y=candidate["y"],
                               rejected_reason="map_boundary", candidate_rejection_reason="map_boundary",
                               planner_feasible=False)
                continue
            if candidate_key in failed_candidate_keys:
                rejected_reason = "previously_failed_in_retry_cycle"
                self.eval_emit(
                    "candidate_checked",
                    target_id=target_id, attempt=attempt, candidate_id=candidate_id,
                    angle_deg=candidate["angle_deg"], radius_m=candidate["radius"],
                    x=candidate["x"], y=candidate["y"],
                    nav_precheck_safe=bool(candidate["nav_precheck_safe"]),
                    uav_safe=bool(candidate["uav_safe"]), planner_feasible=False,
                    path_length_m=None, planning_time_s=0.0, rejected_reason=rejected_reason,
                )
                self.get_logger().info(
                    f"Candidate {i} skipped because it already failed in this retry cycle"
                )
                continue

            if not candidate["uav_safe"]:
                rejected_reason = "uav_safety_check"
                reason = (
                    f"candidate={i}, angle={candidate['angle_deg']}, "
                    f"radius={candidate['radius']:.2f}: "
                    "rejected by UAV safety check"
                )
                self.current_candidate_rejections.append(reason)
                self.eval_emit(
                    "candidate_checked",
                    target_id=target_id, attempt=attempt, candidate_id=candidate_id,
                    angle_deg=candidate["angle_deg"], radius_m=candidate["radius"],
                    x=candidate["x"], y=candidate["y"],
                    nav_precheck_safe=bool(candidate["nav_precheck_safe"]),
                    uav_safe=False, planner_feasible=False, path_length_m=None,
                    planning_time_s=0.0, rejected_reason=rejected_reason,
                )
                continue

            if not candidate["nav_precheck_safe"]:
                self.get_logger().info(
                    f"Candidate {i} Nav2 precheck was conservative; "
                    "requesting an authoritative planner result"
                )

            approach_pose = self.make_pose(candidate["x"], candidate["y"], candidate["yaw"])
            path = self.get_feasible_path_to_pose(approach_pose)
            planning_time_s = time.monotonic() - planning_start
            self.eval_planning_time_accumulator += planning_time_s

            if path is None:
                if not rclpy.ok():
                    return []

                planner_reason = self.last_planner_failure_reason or "nav2_planner_no_path"

                # A safely recovered response-timeout is an infrastructure
                # interruption, not evidence that this geometric candidate is
                # infeasible. Retry this SAME candidate once after cleanup.
                if planner_reason == "planner_goal_response_timeout_recovered":
                    retry_start = time.monotonic()
                    path = self.get_feasible_path_to_pose(approach_pose)
                    retry_time = time.monotonic() - retry_start
                    planning_time_s += retry_time
                    self.eval_planning_time_accumulator += retry_time
                    planner_reason = self.last_planner_failure_reason or planner_reason

                if path is None and self.planner_failure_is_infrastructure(planner_reason):
                    self.last_candidate_batch_infrastructure_reason = planner_reason
                    self.get_logger().error(
                        f"Candidate batch interrupted by planner infrastructure "
                        f"state at candidate {i}: {planner_reason}. Candidate is "
                        "NOT counted as infeasible."
                    )
                    return []

                if path is None:
                    reason = (
                        f"candidate={i}, angle={candidate['angle_deg']}, "
                        f"radius={candidate['radius']:.2f}: {planner_reason}"
                    )
                    self.current_candidate_rejections.append(reason)
                    self.eval_emit(
                        "candidate_checked",
                        target_id=target_id, attempt=attempt, candidate_id=candidate_id,
                        angle_deg=candidate["angle_deg"], radius_m=candidate["radius"],
                        x=candidate["x"], y=candidate["y"],
                        nav_precheck_safe=bool(candidate["nav_precheck_safe"]),
                        uav_safe=bool(candidate["uav_safe"]), planner_feasible=False,
                        path_length_m=None, planning_time_s=planning_time_s,
                        rejected_reason=planner_reason,
                    )
                    self.get_logger().warn(f"Candidate {i} rejected: {planner_reason}")
                    continue

            path_len = self.compute_path_length(path)
            planner_feasible = True
            self.eval_emit(
                "candidate_checked",
                target_id=target_id, attempt=attempt, candidate_id=candidate_id,
                angle_deg=candidate["angle_deg"], radius_m=candidate["radius"],
                x=candidate["x"], y=candidate["y"],
                nav_precheck_safe=bool(candidate["nav_precheck_safe"]),
                uav_safe=bool(candidate["uav_safe"]), planner_feasible=True,
                path_length_m=path_len, planning_time_s=planning_time_s,
                rejected_reason="",
            )
            feasible_candidates.append((path_len, i, candidate, approach_pose))
            self.get_logger().info(
                f"Candidate {i} accepted by planner. path_length={path_len:.2f} m"
            )

            # Continue through the full bounded 32-candidate set. We keep only
            # the best few feasible candidates for execution, but all candidates
            # are logged so candidate feasibility/filtering statistics are valid.

        # A planner goal-response timeout can leave a late action response
        # in flight. Do NOT launch NavigateToPose using cached candidates until
        # that planner action has either been rejected or accepted-and-canceled.
        if self.planner_submission_uncertain:
            if not self.wait_for_planner_transport_idle():
                self.last_planner_failure_reason = "planner_transport_cleanup_timeout"
                self.last_candidate_batch_infrastructure_reason = (
                    "planner_transport_cleanup_timeout"
                )
                self.get_logger().error(
                    "Candidate batch cannot transition to navigation because "
                    "planner transport cleanup is still unresolved."
                )
                return []

        # Prefer the shortest safe route. Ring radius is only a tie-breaker so
        # a slightly closer service point cannot force a much longer detour.
        feasible_candidates.sort(key=lambda item: (item[0], item[2]["radius"]))
        return feasible_candidates[: self.max_candidates_to_execute_per_cycle]

    def execute_one_replan_cycle_for_current_target(self, target: Dict[str, float]) -> bool:
        target_x = target["x"]
        target_y = target["y"]

        # RC11 state-machine guard:
        # Candidate planning must NEVER run while a previous NavigateToPose
        # action is still active or waiting for cancellation to become terminal.
        #
        # Returning here does not consume a candidate cycle or a retry.
        # The existing pending-cancellation handling is allowed to finish first.
        if self.active_navigation_goal or self.navigation_cancel_pending:
            self.get_logger().warn(
                f"Candidate planning deferred for target_id={target.get('id')}: "
                f"NavigateToPose is still "
                f"{'ACTIVE' if self.active_navigation_goal else 'CANCELING'}. "
                "Waiting for terminal action state before replanning."
            )
            return False

        target["candidate_cycles"] += 1
        self.current_candidate_rejections = []
        self.last_target_failure_reason = ""
        self.start_pose_recovery_count = 0
        self.eval_active_candidate_id = None
        self.eval_active_candidate_index = None
        self.eval_active_candidate_angle = None
        self.eval_active_candidate_radius = None

        if self.target_is_treated(target_x, target_y):
            self.get_logger().info(
                f"Already within treatment reach of current target: "
                f"{self.distance_to(target_x, target_y):.2f} m"
            )
            return True

        failed_candidate_keys: Set[Tuple[float, int]] = set()
        maximum_attempts = min(
            self.max_replan_rounds_per_cycle,
            self.max_candidates_to_execute_per_cycle,
        )
        # Use TargetManager's stable ID for every quantitative event so all
        # events for one target aggregate into exactly one target record.
        target_id = str(target["id"])

        # The full bounded 32-candidate set is evaluated ONCE per candidate
        # cycle. The best few candidates returned by plan_feasible_candidates()
        # are then consumed by the bounded execution attempts.
        #
        # This matches the intended design:
        #   32 candidate evaluations -> rank -> best 3 execution attempts.
        candidate_plan_cache = None

        for attempt_index in range(1, maximum_attempts + 1):
            self.eval_target_attempt = attempt_index
            self.eval_active_target_id = str(target.get("id", target_id))
            self.eval_planning_time_accumulator = 0.0

            if candidate_plan_cache is None:
                self.get_logger().info(
                    f"Planning full bounded candidate set once for "
                    f"target_id={target_id}: 32 candidates, "
                    f"execution_attempts={maximum_attempts}"
                )

                feasible_candidates = self.plan_feasible_candidates(
                    target_x,
                    target_y,
                    failed_candidate_keys=failed_candidate_keys,
                    target_id=target_id,
                    attempt=attempt_index,
                )

                if self.last_candidate_batch_infrastructure_reason:
                    # No method outcome occurred: do not consume a target cycle
                    # or no-candidate budget because transport, not geometry,
                    # interrupted the batch.
                    target["candidate_cycles"] = max(0, target["candidate_cycles"] - 1)
                    self.last_target_failure_reason = (
                        f"infrastructure:{self.last_candidate_batch_infrastructure_reason}"
                    )
                    return False

                candidate_plan_cache = list(feasible_candidates)

                self.get_logger().info(
                    f"Candidate planning batch complete for target_id={target_id}: "
                    f"ranked_execution_candidates={len(candidate_plan_cache)}"
                )

            else:
                # Do not run the global planner across all 32 candidates again.
                # Consume the remaining ranked candidates from the same bounded
                # planning batch.
                feasible_candidates = []

                for item in candidate_plan_cache:
                    _, _, cached_candidate, _ = item
                    cached_key = (
                        round(cached_candidate["radius"], 3),
                        int(cached_candidate["angle_deg"]),
                    )

                    if cached_key not in failed_candidate_keys:
                        feasible_candidates.append(item)

                self.get_logger().info(
                    f"Reusing ranked candidate batch for target_id={target_id}, "
                    f"attempt={attempt_index}/{maximum_attempts}, "
                    f"remaining={len(feasible_candidates)}, "
                    f"failed_candidates={len(failed_candidate_keys)}"
                )

            if not feasible_candidates:
                target["no_candidate_cycles"] += 1
                self.last_target_failure_reason = (
                    f"No feasible candidate; "
                    f"rejections={len(self.current_candidate_rejections)}"
                )
                self.get_logger().error(
                    f"No unfailed feasible candidate remains for target_id={target_id}. "
                    f"Candidate cycle failed after {len(failed_candidate_keys)} attempts."
                )
                if (
                    target["no_candidate_cycles"]
                    >= self.max_consecutive_no_candidate_cycles
                ):
                    self.get_logger().error(
                        f"Target candidate search exhausted for target_id={target_id} "
                        f"after {target['no_candidate_cycles']} consecutive "
                        "zero-candidate cycles."
                    )
                else:
                    self.clear_costmaps_safe()
                return False

            target["no_candidate_cycles"] = 0

            path_len, candidate_number, candidate, approach_pose = feasible_candidates[0]
            candidate_key = (
                round(candidate["radius"], 3),
                int(candidate["angle_deg"]),
            )

            # After a failed attempt the robot may have moved. Revalidate only
            # the selected cached retry candidate from the CURRENT pose; do not
            # rerun all 32 candidates inside the same bounded cycle.
            if attempt_index > 1:
                revalidation_start = time.monotonic()
                refreshed_path = self.get_feasible_path_to_pose(
                    approach_pose
                )
                revalidation_time = (
                    time.monotonic() - revalidation_start
                )
                self.eval_planning_time_accumulator += revalidation_time

                if refreshed_path is None:
                    planner_reason = (
                        self.last_planner_failure_reason
                        or "retry_candidate_revalidation_failed"
                    )
                    if planner_reason == "planner_goal_response_timeout_recovered":
                        refreshed_path = self.get_feasible_path_to_pose(approach_pose)
                        planner_reason = self.last_planner_failure_reason or planner_reason

                    if refreshed_path is None and self.planner_failure_is_infrastructure(planner_reason):
                        self.last_target_failure_reason = f"infrastructure:{planner_reason}"
                        self.set_target_state(target, self.TARGET_ACTIVE)
                        return False

                    if refreshed_path is None:
                        failed_candidate_keys.add(candidate_key)
                        self.eval_emit(
                            "replan",
                            target_id=target_id,
                            attempt=attempt_index,
                            failed_candidate_id=candidate["candidate_id"],
                            candidates_excluded=len(failed_candidate_keys),
                            candidate_count=self.max_candidates_checked,
                            reason=planner_reason,
                        )
                        self.get_logger().warn(
                            f"Cached candidate {candidate_number} is no longer "
                            f"feasible from the current pose: {planner_reason}"
                        )
                        continue

                path_len = self.compute_path_length(refreshed_path)
                self.get_logger().info(
                    f"Cached candidate {candidate_number} revalidated from "
                    f"current pose: path={path_len:.2f} m"
                )

            selected_rank = 1
            if candidate_plan_cache is not None:
                for rank, cached_item in enumerate(
                    candidate_plan_cache,
                    start=1,
                ):
                    if (
                        cached_item[2]["candidate_id"]
                        == candidate["candidate_id"]
                    ):
                        selected_rank = rank
                        break

            timeout = min(
                self.max_candidate_timeout_sec,
                max(self.min_candidate_timeout_sec, 30.0 + path_len * self.seconds_per_meter),
            )

            self.eval_active_candidate_id = candidate["candidate_id"]
            self.eval_active_candidate_index = candidate_number
            self.eval_active_candidate_angle = candidate["angle_deg"]
            self.eval_active_candidate_radius = candidate["radius"]
            self.eval_emit(
                "candidate_selected",
                target_id=target_id, attempt=attempt_index,
                candidate_id=candidate["candidate_id"], candidate_index=candidate_number,
                angle_deg=candidate["angle_deg"], radius_m=candidate["radius"],
                x=candidate["x"], y=candidate["y"], path_length_m=path_len,
                planning_time_s=self.eval_planning_time_accumulator,
                selected_rank=selected_rank, navigation_timeout_s=timeout,
            )
            self.get_logger().info(
                f"Candidate selected: target_id={target_id}, "
                f"attempt={attempt_index}/{maximum_attempts}, "
                f"candidate={candidate_number}, angle={candidate['angle_deg']} deg, "
                f"radius={candidate['radius']:.2f}, x={candidate['x']:.2f}, "
                f"y={candidate['y']:.2f}, path={path_len:.2f} m, "
                f"timeout={timeout:.1f}s"
            )

            self.set_target_state(target, self.TARGET_NAVIGATING)
            reached = self.go_to_pose_and_wait(
                approach_pose,
                f"same target approach candidate {candidate_number}",
                acceptance_radius=self.service_acceptance_radius,
                timeout_sec=timeout,
                target_id=target_id,
                candidate_index=candidate_number,
                attempt=attempt_index,
                target_center=(target_x, target_y),
                allow_front_obstacle_replan=True,
                enable_stuck_recovery=True,
            )

            if (
                not reached
                and (
                    self.active_navigation_goal
                    or self.navigation_cancel_pending
                )
            ):
                self.last_target_failure_reason = (
                    "NavigateToPose cancellation pending; candidate execution "
                    "deferred until the previous action reaches terminal state"
                )

                self.get_logger().warn(
                    f"Candidate cycle paused for target_id={target_id}: "
                    "previous NavigateToPose is still ACTIVE/CANCELING. "
                    "No candidate is marked failed and no new planning or goal "
                    "submission is permitted until terminal state."
                )

                return False

            if reached:
                target["infrastructure_retries"] = 0
                self.get_logger().info(
                    f"Current target reached by candidate {candidate_number}"
                )
                return True

            if self.navigation_failure_is_infrastructure(self.last_navigation_reason):
                self.set_target_state(target, self.TARGET_ACTIVE)
                self.last_target_failure_reason = (
                    f"infrastructure:{self.last_navigation_reason}"
                )
                self.get_logger().warn(
                    f"Candidate {candidate_number} is NOT marked failed because "
                    f"navigation transport reported {self.last_navigation_reason}."
                )
                return False

            localization_interrupt_reasons = {
                "amcl_stale",
                "amcl_missing",
                "amcl_covariance_high",
                "tf_stale",
                "tf_missing",
                "tf_amcl_mismatch",
            }
            if self.last_navigation_reason in localization_interrupt_reasons:
                self.set_target_state(target, self.TARGET_ACTIVE)
                self.last_target_failure_reason = (
                    f"Localization interrupted navigation: "
                    f"{self.last_navigation_reason}"
                )
                self.get_logger().warn(
                    f"Candidate {candidate_number} is NOT marked failed because "
                    f"navigation stopped for localization reason "
                    f"{self.last_navigation_reason}."
                )
                return False

            self.set_target_state(target, self.TARGET_ACTIVE)
            target["infrastructure_retries"] = 0
            failed_candidate_keys.add(candidate_key)
            self.eval_emit(
                "replan",
                target_id=target_id, attempt=attempt_index,
                failed_candidate_id=candidate["candidate_id"],
                candidates_excluded=len(failed_candidate_keys),
                candidate_count=self.max_candidates_checked, reason="candidate_failed",
            )
            self.get_logger().warn(
                f"Candidate failed: target_id={target_id}, "
                f"attempt={attempt_index}/{maximum_attempts}, "
                f"candidate={candidate_number}. It is excluded from this retry cycle."
            )

            if self.navigation_lifecycle_fault:
                self.last_target_failure_reason = (
                    "NavigateToPose lifecycle failure"
                )
                return False

        self.last_target_failure_reason = (
            f"All {maximum_attempts} candidate attempts failed"
        )
        self.get_logger().error(
            f"All {maximum_attempts} prepared candidate attempts failed for "
            f"target_id={target_id}"
        )
        return False

    def work_on_current_target_until_treated(self, target: Dict[str, float]) -> bool:
        self.get_logger().info(
            "Starting one bounded candidate cycle for the active target"
        )

        self.get_logger().info(
            f"Starting bounded candidate cycle: target center "
            f"x={target['x']:.2f}, y={target['y']:.2f}"
        )

        reached = self.execute_one_replan_cycle_for_current_target(target)
        if reached:
            return True

        if self.navigation_lifecycle_fault:
            self.get_logger().error(
                "Current target stopped because navigation action lifecycle handling failed."
            )
        elif self.mission_halted:
            self.get_logger().error(
                "Current target remains active, but the mission is safely halted "
                "after repeated zero-candidate cycles."
            )
        elif self.active_navigation_goal or self.navigation_cancel_pending:
            self.get_logger().warn(
                "Current target cycle paused while NavigateToPose reaches a "
                "terminal state; no new planning will start meanwhile."
            )
        elif self.last_navigation_reason in {
            "amcl_stale",
            "amcl_missing",
            "amcl_covariance_high",
            "tf_stale",
            "tf_missing",
            "tf_amcl_mismatch",
        }:
            self.get_logger().warn(
                f"Current target cycle paused by localization: "
                f"{self.last_navigation_reason}. Candidate identity is preserved."
            )
        else:
            self.get_logger().error(
                "Current target candidate cycle exhausted. "
                "The same target remains active for a fresh bounded cycle."
            )

        self.stop_robot()
        return False

    def navigation_timeout_for_distance(self, goal_x: float, goal_y: float) -> float:
        distance = self.distance_to(goal_x, goal_y)
        return min(
            self.max_candidate_timeout_sec,
            max(self.min_candidate_timeout_sec, 30.0 + distance * self.seconds_per_meter),
        )

    def complete_treatment_for_target(self, target: Dict[str, float]) -> bool:
        target_distance = self.distance_to(target["x"], target["y"])

        # RC12 final invariant guard. No target may be treated unless the
        # physical target distance is within the frozen treatment radius at
        # the instant treatment is about to begin.
        if target_distance > self.treatment_reach_radius:
            self.last_target_failure_reason = (
                "treatment_invariant_violation: "
                f"distance={target_distance:.6f}m > "
                f"limit={self.treatment_reach_radius:.6f}m"
            )
            self.get_logger().error(
                f"Treatment invariant rejected target={target['id']}: "
                f"target distance={target_distance:.6f} m > "
                f"{self.treatment_reach_radius:.6f} m. "
                "Treatment will NOT be counted as successful."
            )
            self.stop_robot()
            return False

        self.get_logger().info(
            f"Treatment started: target distance={target_distance:.2f} m "
            f"(allowed <= {self.treatment_reach_radius:.2f} m)"
        )
        self.stop_robot()
        treatment_start_sim = self.eval_sim_time()
        self.spin_for_duration(self.treatment_seconds)
        treatment_duration_sim = self.eval_sim_time() - treatment_start_sim
        self.eval_emit(
            "treatment_completed",
            target_id=target["id"], attempt=self.eval_target_attempt,
            target_x=float(target["x"]), target_y=float(target["y"]),
            treatment_duration_s=treatment_duration_sim,
            treatment_sim_duration_s=treatment_duration_sim, target_success=True,
        )
        self.get_logger().info(
            f"Treatment finished successfully: id={target['id']}"
        )

        self.set_target_state(target, self.TARGET_TREATED)
        self.completed_targets.append(target)
        self.current_target = None
        self.current_candidate_rejections = []
        self.last_target_failure_reason = ""
        self.get_logger().info(
            f"Target treated: id={target['id']}. "
            f"Continuing directly to pending targets={len(self.target_queue)}"
        )
        return True

    def fail_current_target_terminal(self, target: Dict[str, float], reason: str):
        target["failure_reason"] = reason
        target["failure_history"].append(
            {
                "pass": self.deferred_retry_pass,
                "reason": reason,
                "candidate_rejections": list(self.current_candidate_rejections),
            }
        )
        self.set_target_state(target, self.TARGET_FAILED)
        self.eval_emit(
            "target_failed",
            target_id=target["id"], attempt=self.eval_target_attempt,
            target_x=float(target["x"]), target_y=float(target["y"]),
            failure_reason=reason,
            candidate_rejections=list(self.current_candidate_rejections),
        )
        self.failed_targets.append(target)
        self.current_target = None
        self.current_candidate_rejections = []
        self.last_target_failure_reason = ""
        self.get_logger().error(
            f"Target failed: id={target['id']}, reason={reason}. "
            "Continuing to remaining targets."
        )
        self.stop_robot()

    def execute_direct_target_baseline(self, target: Dict[str, float]) -> bool:
        target_id = str(target["id"])
        target_x = float(target["x"])
        target_y = float(target["y"])
        self.eval_active_target_id = target_id
        self.eval_target_attempt = 1
        self.eval_active_candidate_id = None
        self.eval_active_candidate_index = None
        self.eval_planning_time_accumulator = 0.0

        if self.target_is_treated(target_x, target_y):
            return True

        goal_pose = self.make_pose(target_x, target_y, self.current_yaw)
        timeout = self.navigation_timeout_for_distance(target_x, target_y)
        self.set_target_state(target, self.TARGET_NAVIGATING)
        reached = self.go_to_pose_and_wait(
            goal_pose,
            "direct target goal",
            acceptance_radius=self.service_acceptance_radius,
            timeout_sec=timeout,
            target_id=target_id,
            candidate_index=None,
            attempt=1,
            target_center=(target_x, target_y),
            allow_front_obstacle_replan=False,
            enable_stuck_recovery=False,
        )
        if reached:
            return True

        self.last_target_failure_reason = (
            f"Direct target goal failed; reason={self.last_navigation_reason}"
        )
        return False

    def execute_single_candidate_baseline(self, target: Dict[str, float]) -> bool:
        target_id = str(target["id"])
        target_x = float(target["x"])
        target_y = float(target["y"])
        self.eval_active_target_id = target_id
        self.eval_target_attempt = 1
        self.eval_active_candidate_index = 1
        self.eval_planning_time_accumulator = 0.0

        if self.target_is_treated(target_x, target_y):
            return True

        candidate = self.compute_single_candidate_approach_pose(target_x, target_y)
        candidate_id = candidate["candidate_id"]
        self.eval_active_candidate_id = candidate_id
        self.eval_active_candidate_angle = candidate["angle_deg"]
        self.eval_active_candidate_radius = candidate["radius"]

        rejected_reason = ""
        path_len = None
        planner_feasible = False
        planning_start = time.monotonic()

        if not self.is_pose_inside_navigable_bounds(candidate["x"], candidate["y"]):
            rejected_reason = "map_boundary"
        elif not candidate["uav_safe"]:
            rejected_reason = "uav_safety_check"
        else:
            approach_pose = self.make_pose(candidate["x"], candidate["y"], candidate["yaw"])
            path = self.get_feasible_path_to_pose(approach_pose)
            if path is None:
                rejected_reason = "nav2_planner_no_path"
            else:
                planner_feasible = True
                path_len = self.compute_path_length(path)

        planning_time_s = time.monotonic() - planning_start
        self.eval_planning_time_accumulator = planning_time_s
        self.eval_emit(
            "candidate_checked",
            target_id=target_id, attempt=1, candidate_id=candidate_id,
            angle_deg=candidate["angle_deg"], radius_m=candidate["radius"],
            x=candidate["x"], y=candidate["y"],
            nav_precheck_safe=bool(candidate["nav_precheck_safe"]),
            uav_safe=bool(candidate["uav_safe"]),
            planner_feasible=bool(planner_feasible),
            path_length_m=path_len,
            planning_time_s=planning_time_s,
            rejected_reason=rejected_reason,
        )

        if not planner_feasible:
            self.last_target_failure_reason = (
                f"Single candidate infeasible; rejected_reason={rejected_reason}"
            )
            self.current_candidate_rejections.append(self.last_target_failure_reason)
            return False

        timeout = min(
            self.max_candidate_timeout_sec,
            max(self.min_candidate_timeout_sec, 30.0 + float(path_len) * self.seconds_per_meter),
        )
        self.eval_emit(
            "candidate_selected",
            target_id=target_id, attempt=1,
            candidate_id=candidate_id, candidate_index=1,
            angle_deg=candidate["angle_deg"], radius_m=candidate["radius"],
            x=candidate["x"], y=candidate["y"], path_length_m=path_len,
            planning_time_s=planning_time_s,
            selected_rank=1, navigation_timeout_s=timeout,
        )

        approach_pose = self.make_pose(candidate["x"], candidate["y"], candidate["yaw"])
        self.set_target_state(target, self.TARGET_NAVIGATING)
        reached = self.go_to_pose_and_wait(
            approach_pose,
            "single fixed candidate",
            acceptance_radius=self.service_acceptance_radius,
            timeout_sec=timeout,
            target_id=target_id,
            candidate_index=1,
            attempt=1,
            target_center=(target_x, target_y),
            allow_front_obstacle_replan=False,
            enable_stuck_recovery=False,
        )
        if reached:
            return True

        self.last_target_failure_reason = (
            f"Single candidate navigation failed; reason={self.last_navigation_reason}"
        )
        return False

    def execute_sequential_nonpersistent_baseline(self, target: Dict[str, float]) -> bool:
        self.get_logger().info(
            "B3 sequential baseline: one bounded candidate-planning pass; "
            "failed targets are not preserved or revisited"
        )
        reached = self.execute_one_replan_cycle_for_current_target(target)
        if reached:
            return True

        reason = self.last_target_failure_reason or "B3 one-pass candidate cycle failed"
        self.last_target_failure_reason = f"B3 sequential/nonpersistent failure: {reason}"
        return False

    def work_on_current_target_for_method(self, target: Dict[str, float]) -> bool:
        if self.eval_method == "B1":
            return self.execute_direct_target_baseline(target)
        if self.eval_method == "B2":
            return self.execute_single_candidate_baseline(target)
        if self.eval_method == "B3":
            return self.execute_sequential_nonpersistent_baseline(target)
        return self.work_on_current_target_until_treated(target)

    def defer_current_target(self, reason: str):
        if self.current_target is None:
            return

        target = self.current_target
        target["failure_reason"] = reason
        target["failure_history"].append(
            {
                "pass": self.deferred_retry_pass,
                "reason": reason,
                "candidate_rejections": list(self.current_candidate_rejections),
            }
        )
        self.set_target_state(target, self.TARGET_DEFERRED)
        self.eval_emit(
            "target_deferred",
            target_id=target["id"], attempt=self.eval_target_attempt,
            target_x=float(target["x"]), target_y=float(target["y"]),
            deferral_reason=reason, candidate_rejections=list(self.current_candidate_rejections),
        )
        self.deferred_targets.append(target)
        self.current_target = None
        self.current_candidate_rejections = []
        self.get_logger().warn(
            f"Target deferred: id={target['id']}, reason={reason}, "
            f"pending={len(self.target_queue)}, "
            f"deferred={len(self.deferred_targets)}"
        )

    def start_deferred_retry_pass(self) -> bool:
        started, retry_targets = self.target_manager.start_deferred_retry_pass(
            self.max_deferred_retry_passes
        )
        if not started:
            return False

        for target in retry_targets:
            self.set_target_state(target, self.TARGET_QUEUED)

        self.get_logger().warn(
            f"Starting deferred retry pass "
            f"{self.deferred_retry_pass}/{self.max_deferred_retry_passes}: "
            f"targets={len(retry_targets)}"
        )
        return True

    def finalize_deferred_targets(self):
        finalized_targets = self.target_manager.finalize_deferred_targets(
            self.max_deferred_retry_passes
        )
        for target in finalized_targets:
            self.eval_emit(
                "target_failed",
                target_id=target["id"], attempt=self.eval_target_attempt,
                target_x=float(target["x"]), target_y=float(target["y"]),
                failure_reason=target.get("failure_reason", "permanently_unreachable"),
            )
            self.get_logger().error(
                f"Target permanently unreachable: id={target['id']}, "
                f"x={target['x']:.2f}, y={target['y']:.2f}, "
                f"reason={target['failure_reason']}"
            )

    def mission_ready_for_return_home(self) -> bool:
        return self.target_manager.mission_ready_for_return_home(
            self.active_navigation_goal,
            self.manual_recovery_active,
        )

    def report_mission_summary(self):
        if self.mission_summary_reported:
            return

        untreated = self.target_manager.untreated_targets()
        self.get_logger().info(
            f"Mission summary: discovered={len(self.targets_by_id)}, "
            f"treated={len(self.completed_targets)}, "
            f"unreachable={len(self.permanently_unreachable_targets)}, "
            f"failed={len(self.failed_targets)}"
        )
        for target in untreated:
            self.get_logger().warn(
                f"Untreated target: id={target['id']}, "
                f"x={target['x']:.2f}, y={target['y']:.2f}, "
                f"state={target['state']}, "
                f"reason={target['failure_reason']}"
            )

        self.mission_summary_reported = True

    # ------------------------------------------------------------------
    # MISSION FLOW
    # ------------------------------------------------------------------

    def process_next_target(self) -> bool:
        if self.finish_pending_cancel_recovery():
            return False

        target = self.select_target_for_method()

        if target is None:
            return False

        target_x = target["x"]
        target_y = target["y"]

        self.get_logger().info("────────────────────────────")
        self.get_logger().info(f"ACTIVE TARGET from {target['source']}")
        self.get_logger().info(f"Target center: x={target_x:.2f}, y={target_y:.2f}")

        reached = self.work_on_current_target_for_method(target)

        if reached:
            if self.complete_treatment_for_target(target):
                return True
            # Final treatment invariant rejected the apparent navigation
            # success. Fall through to the existing method-specific
            # failure/retry/defer policy using last_target_failure_reason.

        reason = self.last_target_failure_reason or "candidate cycle failed"

        if reason.startswith("infrastructure:"):
            infra_reason = reason.split(":", 1)[1]
            target["infrastructure_retries"] = int(
                target.get("infrastructure_retries", 0)
            ) + 1
            self.set_target_state(target, self.TARGET_ACTIVE)
            self.current_candidate_rejections = []

            fatal_transport = self.mission_halted or self.navigation_lifecycle_fault
            if (
                fatal_transport
                or target["infrastructure_retries"] > self.max_infrastructure_retries_per_target
            ):
                self.infrastructure_fault_reason = infra_reason
                self.mission_halted = True
                self.eval_emit(
                    "infrastructure_fault",
                    target_id=target["id"],
                    reason=infra_reason,
                    retry_count=int(target["infrastructure_retries"]),
                )
                self.get_logger().error(
                    f"Mission halted on unresolved/bounded infrastructure fault for "
                    f"target_id={target['id']}: {infra_reason}. Target is "
                    "preserved and is NOT classified as geometrically failed."
                )
                self.stop_robot()
                return False

            self.get_logger().warn(
                f"Infrastructure interruption for target_id={target['id']}: "
                f"{infra_reason}; bounded retry "
                f"{target['infrastructure_retries']}/"
                f"{self.max_infrastructure_retries_per_target}. "
                "No candidate/no-path failure is counted."
            )
            self.stop_robot()
            self.spin_wall_duration(min(1.0, self.replan_pause_sec))
            return False

        if self.eval_method in {"B1", "B2", "B3"}:
            self.fail_current_target_terminal(target, reason)
            return False

        if self.navigation_lifecycle_fault:
            target["failure_reason"] = reason
            target["failure_history"].append(
                {
                    "pass": self.deferred_retry_pass,
                    "reason": reason,
                    "candidate_rejections": list(self.current_candidate_rejections),
                }
            )
            self.set_target_state(target, self.TARGET_FAILED)
            self.eval_emit(
                "target_failed",
                target_id=target["id"], attempt=self.eval_target_attempt,
                target_x=float(target["x"]), target_y=float(target["y"]),
                failure_reason=reason,
            )
            self.failed_targets.append(target)
            self.current_target = None
            self.mission_halted = True
            self.get_logger().error(
                "Mission halted because NavigateToPose lifecycle safety "
                "could not be guaranteed"
            )
            self.stop_robot()
            return False

        target["failure_reason"] = reason
        target["failure_history"].append(
            {
                "pass": self.deferred_retry_pass,
                "reason": reason,
                "candidate_rejections": list(self.current_candidate_rejections),
            }
        )
        self.set_target_state(target, self.TARGET_ACTIVE)

        target_cycle_limit_reached = (
            target["candidate_cycles"] >= self.max_target_candidate_cycles
            or target["no_candidate_cycles"] >= self.max_consecutive_no_candidate_cycles
        )
        if target_cycle_limit_reached:
            bounded_reason = (
                f"{reason}; target retry limit reached "
                f"(candidate_cycles={target['candidate_cycles']}/"
                f"{self.max_target_candidate_cycles}, "
                f"no_candidate_cycles={target['no_candidate_cycles']}/"
                f"{self.max_consecutive_no_candidate_cycles})"
            )
            target["failure_reason"] = bounded_reason
            self.set_target_state(target, self.TARGET_FAILED)
            self.eval_emit(
                "target_failed",
                target_id=target["id"], attempt=self.eval_target_attempt,
                target_x=float(target["x"]), target_y=float(target["y"]),
                failure_reason=bounded_reason,
                candidate_cycles=int(target["candidate_cycles"]),
                max_target_candidate_cycles=int(self.max_target_candidate_cycles),
                no_candidate_cycles=int(target["no_candidate_cycles"]),
                max_consecutive_no_candidate_cycles=int(self.max_consecutive_no_candidate_cycles),
                candidate_rejections=list(self.current_candidate_rejections),
            )
            self.failed_targets.append(target)
            self.current_target = None
            self.current_candidate_rejections = []
            self.last_target_failure_reason = ""
            self.get_logger().error(
                f"Target failed after bounded retry policy: id={target['id']}, "
                f"reason={bounded_reason}. Continuing to remaining targets."
            )
            self.stop_robot()
            return False

        self.current_candidate_rejections = []

        if self.mission_halted:
            self.eval_emit(
                "target_failed",
                target_id=target["id"], attempt=self.eval_target_attempt,
                target_x=float(target["x"]), target_y=float(target["y"]),
                failure_reason=reason,
            )
            self.get_logger().error(
                f"Current target preserved while halted: id={target['id']}, "
                f"pending_targets_held={len(self.target_queue)}"
            )
            self.stop_robot()
            return False

        self.get_logger().warn(
            f"Retrying the same active target after a bounded pause: "
            f"id={target['id']}, completed_cycles={target['candidate_cycles']}, "
            f"pending_targets_held={len(self.target_queue)}"
        )
        self.stop_robot()
        self.spin_for_duration(self.replan_pause_sec)
        return False

    def clear_local_costmap_bounded(self):
        client = self.clear_costmap_local_srv
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                "Local costmap clear service unavailable after 1.0 s"
            )
            return False
        future = client.call_async(ClearEntireCostmap.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if not future.done() or future.exception() is not None:
            self.get_logger().warn(
                "Local costmap clear timed out or failed"
            )
            return False
        return True

    def clear_global_costmap_bounded(self):
        client = self.clear_costmap_global_srv
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                "Global costmap clear service unavailable after 1.0 s"
            )
            return False
        future = client.call_async(ClearEntireCostmap.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if not future.done() or future.exception() is not None:
            self.get_logger().warn(
                "Global costmap clear timed out or failed"
            )
            return False
        return True

    def footprint_max_cost(self, grid, x, y, yaw):
        if grid is None or grid.info.resolution <= 0:
            return None
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            return None

        # Conservative cell-center test with half-cell-diagonal padding.
        # This preserves occupancy safety while using the URDF-derived
        # orientation-aware physical envelope instead of the old 0.38 m circle.
        cell_margin = 0.5 * grid.info.resolution * math.sqrt(2.0)

        c = math.cos(yaw)
        s = math.sin(yaw)

        # Search a yaw-independent bounding region large enough to contain
        # every orientation of the rectangular physical footprint.
        max_corner_radius = max(
            math.hypot(px, py)
            for px, py in self.physical_footprint
        )
        extent = max_corner_radius + cell_margin

        center = self.world_to_grid_index(grid, x, y)
        if center is None:
            return None

        radius_cells = max(1, math.ceil(extent / grid.info.resolution))
        values = []

        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                mx = center[0] + dx
                my = center[1] + dy

                if not (0 <= mx < grid.info.width and 0 <= my < grid.info.height):
                    return None

                info = grid.info
                q = info.origin.orientation
                grid_yaw = math.atan2(
                    2 * (q.w * q.z + q.x * q.y),
                    1 - 2 * (q.y * q.y + q.z * q.z),
                )

                gx = (mx + 0.5) * info.resolution
                gy = (my + 0.5) * info.resolution

                cg = math.cos(grid_yaw)
                sg = math.sin(grid_yaw)

                wx = info.origin.position.x + cg * gx - sg * gy
                wy = info.origin.position.y + sg * gx + cg * gy

                ddx = wx - x
                ddy = wy - y

                # Transform cell center into robot-local coordinates.
                lx = c * ddx + s * ddy
                ly = -s * ddx + c * ddy

                inside = (
                    self.physical_footprint_rear - cell_margin
                    <= lx
                    <= self.physical_footprint_front + cell_margin
                    and
                    self.physical_footprint_right - cell_margin
                    <= ly
                    <= self.physical_footprint_left + cell_margin
                )

                if not inside:
                    continue

                value = self.grid_cell_value(grid, mx, my)
                if value < 0:
                    return None
                values.append(value)

        return max(values) if values else 0

    def exact_physical_footprint_has_lethal_cell(
        self,
        grid: OccupancyGrid,
        x: float,
        y: float,
        yaw: float,
        lethal_threshold: int,
    ):
        """Return True only for lethal cells inside the true RC2 footprint.

        Unlike footprint_max_cost(), this helper intentionally applies no
        half-cell padding and is used only by the RC8 pre-home start-state
        collision classification.
        """
        if grid is None or grid.info.resolution <= 0:
            return None

        if not all(math.isfinite(v) for v in (x, y, yaw)):
            return None

        center = self.world_to_grid_index(grid, x, y)
        if center is None:
            return None

        c = math.cos(yaw)
        s = math.sin(yaw)

        max_corner_radius = max(
            math.hypot(px, py)
            for px, py in self.physical_footprint
        )
        radius_cells = max(
            1,
            math.ceil(max_corner_radius / grid.info.resolution) + 1,
        )

        info = grid.info
        q = info.origin.orientation
        grid_yaw = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y * q.y + q.z * q.z),
        )
        cg = math.cos(grid_yaw)
        sg = math.sin(grid_yaw)

        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                mx = center[0] + dx
                my = center[1] + dy

                if not (
                    0 <= mx < info.width
                    and 0 <= my < info.height
                ):
                    return None

                value = self.grid_cell_value(grid, mx, my)

                # RC8 exact intersection test:
                # transform all four grid-cell corners into robot-local
                # coordinates, then use SAT against the axis-aligned RC2
                # physical footprint rectangle.
                cell_corners_local = []

                for ox, oy in (
                    (0.0, 0.0),
                    (info.resolution, 0.0),
                    (info.resolution, info.resolution),
                    (0.0, info.resolution),
                ):
                    gx = mx * info.resolution + ox
                    gy = my * info.resolution + oy

                    wx = info.origin.position.x + cg * gx - sg * gy
                    wy = info.origin.position.y + sg * gx + cg * gy

                    ddx = wx - x
                    ddy = wy - y

                    lx = c * ddx + s * ddy
                    ly = -s * ddx + c * ddy

                    cell_corners_local.append((lx, ly))

                footprint = (
                    (self.physical_footprint_rear, self.physical_footprint_right),
                    (self.physical_footprint_front, self.physical_footprint_right),
                    (self.physical_footprint_front, self.physical_footprint_left),
                    (self.physical_footprint_rear, self.physical_footprint_left),
                )

                def polygons_overlap(poly_a, poly_b):
                    for poly in (poly_a, poly_b):
                        for i in range(len(poly)):
                            x1, y1 = poly[i]
                            x2, y2 = poly[(i + 1) % len(poly)]

                            axis_x = -(y2 - y1)
                            axis_y = x2 - x1

                            a_proj = [
                                px * axis_x + py * axis_y
                                for px, py in poly_a
                            ]
                            b_proj = [
                                px * axis_x + py * axis_y
                                for px, py in poly_b
                            ]

                            if max(a_proj) < min(b_proj) or max(b_proj) < min(a_proj):
                                return False

                    return True

                if not polygons_overlap(footprint, cell_corners_local):
                    continue

                # Unknown is unsafe only when that unknown cell actually
                # intersects the physical footprint.
                if value < 0:
                    return None

                if value >= lethal_threshold:
                    return True

        return False

    def planner_start_state(self):
        if not self.drain_localization_tf_callbacks():
            state = "planner_start_unknown"
            diagnostics = self.navigation_diagnostic_snapshot()
        else:
            diagnostics = self.navigation_diagnostic_snapshot()

            ages = [
                diagnostics.get("global_costmap_age_s"),
                diagnostics.get("local_costmap_age_s"),
            ]

            static_collision = self.exact_physical_footprint_has_lethal_cell(
                self.static_map,
                self.current_x,
                self.current_y,
                self.current_yaw,
                100,
            )

            local_collision = None
            if self.local_costmap is not None:
                try:
                    tf = self.tf_buffer.lookup_transform(
                        self.local_costmap.header.frame_id,
                        "base_footprint",
                        rclpy.time.Time(),
                    )
                    q = tf.transform.rotation
                    local_yaw = math.atan2(
                        2 * (q.w * q.z + q.x * q.y),
                        1 - 2 * (q.y * q.y + q.z * q.z),
                    )
                    local_collision = self.exact_physical_footprint_has_lethal_cell(
                        self.local_costmap,
                        tf.transform.translation.x,
                        tf.transform.translation.y,
                        local_yaw,
                        100,
                    )
                except TransformException:
                    local_collision = None

            diagnostics["static_physical_collision"] = static_collision
            diagnostics["local_physical_collision"] = local_collision
            global_footprint_cost = diagnostics.get("global_footprint_max_cost")

            if (
                static_collision is None
                or local_collision is None
                or any(v is None or not -0.3 <= v <= 3.0 for v in ages)
            ):
                state = "planner_start_unknown"
            elif not self.is_pose_inside_navigable_bounds(
                self.current_x,
                self.current_y,
            ):
                state = "planner_start_blocked"
            elif (
                static_collision
                or local_collision
                or global_footprint_cost is None
                or global_footprint_cost >= 99
            ):
                state = "planner_start_blocked"
            else:
                state = "planner_start_safe"

        self.eval_emit(
            "return_home_start_check",
            state=state,
            diagnostics=diagnostics,
        )
        return state

    def pre_home_recovery(self):
        if self.pre_home_recovery_used:
            return False
        self.pre_home_recovery_used = True

        if not self.wait_for_valid_localization():
            return False

        # First handle the same blocked-footprint condition that can occur after
        # treatment. Use reverse egress, never an in-place sweep while blocked.
        self.start_pose_recovery_count = 0
        self.blocked_start_recovery_pose = None
        start_safe, _ = self.current_costmap_status()
        if not start_safe:
            for recovery_index in range(self.max_start_pose_recoveries_per_cycle):
                self.start_pose_recovery_count = recovery_index
                if not self.recover_from_blocked_start_pose("return_home"):
                    return False
                self.spin_wall_duration(0.5)
                start_safe, _ = self.current_costmap_status()
                if start_safe:
                    self.blocked_start_recovery_pose = None
                    break
                self.blocked_start_recovery_pose = None
            if not start_safe:
                return False

        state = self.planner_start_state()
        if state == "planner_start_safe":
            return True

        # If the footprint is clear but the start remains conservatively
        # blocked for another reason, allow one bounded rotation only when its
        # complete physical sweep is safe.
        pre_home_turn_z = self.choose_escape_turn_direction()
        if not (
            self.localization_valid()
            and self.is_pose_inside_navigable_bounds(self.current_x, self.current_y)
            and self.is_rotation_sweep_safe(
                self.current_x, self.current_y, self.current_yaw,
                pre_home_turn_z, 0.55
            )
            and not self.front_obstacle_detected
        ):
            return False

        if not self.begin_manual_recovery("pre-home safe rotation"):
            return False
        twist = Twist()
        twist.angular.z = math.copysign(0.55, pre_home_turn_z)
        started = self.physical_now()
        while rclpy.ok() and self.physical_now() - started < 0.55:
            self.safe_publish_twist(twist)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.04)
        self.end_manual_recovery(stop_repeats=2)
        self.clear_costmaps_safe()
        self.spin_wall_duration(0.5)
        return self.planner_start_state() == "planner_start_safe"

    def final_home_telemetry(self):
        valid = self.drain_localization_tf_callbacks()
        x, y = (self.current_x, self.current_y) if valid else (None, None)
        distance_valid = (self.return_odom_valid and self.return_odom_samples >= 2
                          and self.return_last_odom is not None
                          and -self.localization_future_tolerance_s <= self.eval_sim_time()-self.return_last_odom[0] <= 1.0)
        return {"home_error_m": math.hypot(x-self.home_x,y-self.home_y) if valid and self.home_x is not None and self.home_y is not None else None,
                "final_robot_x":x, "final_robot_y":y, "final_valid_pose_x":x, "final_valid_pose_y":y,
                "home_x":self.home_x, "home_y":self.home_y,
                "localization_source":"amcl_validated_map_tf" if valid else "unavailable",
                "pose_age":self.eval_sim_time()-self.last_valid_tf_sim if valid else None,
                "covariance":self.last_amcl_diagnostic.get("covariance_xy_yaw"),
                "actual_return_path_length_m":self.return_distance if distance_valid else None,
                "actual_return_path_source":"odom" if distance_valid else "unavailable",
                "return_path_source":"odom" if distance_valid else "unavailable",
                "final_localization_reason":self.localization_reason,
                "return_odom_samples":self.return_odom_samples,
                "return_odom_first_stamp":getattr(self,"return_first_odom_sim",None),
                "return_odom_last_stamp":self.return_last_odom[0] if self.return_last_odom else None,
                "return_odom_continuity_valid":self.return_odom_valid}

    def home_path_precheck(self, goal):
        started = time.monotonic()
        self.home_precheck_attempt += 1
        self.home_precheck_failure_reason = None
        path = None
        try:
            if self.planner_start_state() != "planner_start_safe":
                self.home_precheck_failure_reason = "planner_start_unsafe"
            elif not self.is_pose_inside_navigable_bounds(self.current_x,self.current_y):
                self.home_precheck_failure_reason = "start_outside_navigable_bounds"
            elif not self.is_pose_inside_navigable_bounds(goal.pose.position.x,goal.pose.position.y):
                self.home_precheck_failure_reason = "goal_outside_navigable_bounds"
            else:
                path = self._home_path_precheck_impl(goal)
            return path
        except TimeoutError:
            self.home_precheck_failure_reason = "planner_timeout"
            raise
        except Exception:
            self.home_precheck_failure_reason = "planner_exception"
            raise
        finally:
            goal_cell = self.world_to_costmap_index(goal.pose.position.x,goal.pose.position.y)
            start_cell = self.world_to_costmap_index(self.current_x,self.current_y)
            self.eval_emit("return_home_plan", precheck_attempt=self.home_precheck_attempt,
                           start_cost=self.costmap_cell_value(*start_cell) if start_cell is not None else None,
                           goal_cost=self.costmap_cell_value(*goal_cell) if goal_cell is not None else None,
                           home_path_feasible=path is not None,
                           home_planned_path_length_m=self.compute_path_length(path) if path else None,
                           home_planning_time_s=time.monotonic()-started,
                           home_precheck_failure_reason=self.home_precheck_failure_reason,
                           goal_x=goal.pose.position.x,goal_y=goal.pose.position.y,
                           diagnostics=self.navigation_diagnostic_snapshot())

    def _home_path_precheck_impl(self, goal):
        start_pose = self.make_pose(
            self.current_x, self.current_y, self.current_yaw
        )

        path = None
        for planner_try in range(2):
            path = self.compute_path_to_pose_bounded(
                start_pose,
                goal,
                timeout_sec=self.home_planning_timeout_s,
                label="return-home precheck",
            )
            if path is not None:
                break
            if self.last_planner_failure_reason != "planner_goal_response_timeout_recovered":
                break
            self.get_logger().warn(
                "Return-home planner response timeout was safely cleaned up; "
                "retrying the same precheck once."
            )

        if path is None:
            self.home_precheck_failure_reason = (
                self.last_planner_failure_reason or "planner_failed_no_path"
            )
            return None

        if not self.path_is_valid(path):
            self.home_precheck_failure_reason = "path_infeasible_under_safety_checks"
            return None
        return path

    def return_home(self) -> bool:
        self.wait_for_valid_localization()
        for node in ("amcl","planner_server","controller_server","bt_navigator"):
            self.observe_lifecycle(node)
        start = self.eval_sim_time()
        home_goal_status = "not_started"
        home_goal_result = "not_started"
        home_terminal_reason = "not_started"
        home_goal_stage_index = None
        self.home_precheck_attempt = 0
        self.pre_home_recovery_used = False
        self.start_pose_recovery_count = 0
        self.blocked_start_recovery_pose = None
        self.eval_active_target_id = None
        self.eval_active_candidate_id = None
        self.return_distance = 0.0
        self.return_phase_start_sim = start
        self.return_first_odom_sim = None
        self.return_odom_samples = 0
        self.return_odom_valid = True
        self.return_last_odom = None
        self.return_last_xy = (self.current_x, self.current_y) if self.localization_valid() else None
        attempts = replans = 0
        planned = None
        success = False
        reason = "invalid_home_or_localization"
        self.eval_emit("return_home_start", home_x=self.home_x, home_y=self.home_y,
                       home_yaw=self.home_yaw, home_frame="map", home_source=self.home_source,
                       home_stamp_sim=self.home_stamp_sim,diagnostics=self.navigation_diagnostic_snapshot())
        try:
            valid = (self.home_saved and self.localization_valid()
                     and self.home_x is not None and self.home_y is not None
                     and self.is_pose_inside_navigable_bounds(self.home_x, self.home_y)
                     and self.is_pose_inside_navigable_bounds(self.current_x, self.current_y))
            idle = (not self.active_navigation_goal and not self.navigation_cancel_pending
                    and not self.manual_recovery_active and self.current_target is None
                    and not self.eval_cancel_pending
                    and self.mission_ready_for_return_home())
            if not idle:
                reason = "return_home_state_not_idle"
            if not valid:
                reason = self.localization_reason if not self.localization_valid() else "invalid_home_or_bounds"
            if valid and idle:
                home = self.make_pose(self.home_x, self.home_y, self.home_yaw)
                for attempt in range(1, self.max_return_home_attempts + 1):
                    if attempt > 1:
                        if replans >= self.max_return_home_replans:
                            break
                        replans += 1
                    attempts = attempt
                    if not self.wait_for_valid_localization():
                        reason = self.localization_reason
                        break
                    state = self.planner_start_state()
                    if state != "planner_start_safe":
                        if not self.pre_home_recovery():
                            reason = state
                            break
                    path = self.home_path_precheck(home)
                    planned = self.compute_path_length(path) if path else None
                    if path is None:
                        reason = self.home_precheck_failure_reason or "home_path_infeasible"
                        # A bounded local refresh is allowed, never execution of an impossible goal.
                        continue
                    stages = []
                    if self.return_home_multistage and planned > self.return_home_stage_distance_m:
                        # Intermediate comes from the validated global route, not invented coordinates.
                        intermediate = path.poses[len(path.poses)//2]
                        if self.is_point_costmap_safe(intermediate.pose.position.x,intermediate.pose.position.y,
                                                      clearance_radius=self.candidate_clearance_radius):
                            stages.append(intermediate)
                        else:
                            self.eval_emit("return_home_stage",skipped=True,reason="intermediate_costmap_unsafe",
                                           goal_x=intermediate.pose.position.x,goal_y=intermediate.pose.position.y)
                    stages.append(home)
                    for index, stage in enumerate(stages):
                        self.eval_emit("return_home_stage", stage_index=index, attempt_count=attempts,
                                       goal_x=stage.pose.position.x, goal_y=stage.pose.position.y)
                        if not self.wait_for_valid_localization() or self.home_path_precheck(stage) is None:
                            reason = self.localization_reason if not self.localization_valid() else (self.home_precheck_failure_reason or "home_stage_infeasible")
                            break
                        reached = self.go_to_pose_and_wait(
                            stage, "home" if index == len(stages)-1 else "home_intermediate",
                            acceptance_radius=self.home_acceptance_radius if index == len(stages)-1 else 0.25,
                            timeout_sec=self.home_timeout_sec, target_id=None, attempt=attempts,
                            allow_front_obstacle_replan=True, enable_stuck_recovery=False)
                        if self.last_navigation_terminal_status != "not_started":
                            home_goal_status = self.last_navigation_terminal_status
                            home_goal_result = self.last_navigation_terminal_result
                            home_terminal_reason = self.last_navigation_terminal_reason
                            home_goal_stage_index = index
                        reason = "home_tolerance_verified" if reached and index == len(stages)-1 else self.last_navigation_reason
                        if not reached:
                            break
                    else:
                        success = self.localization_valid() and self.distance_to(self.home_x, self.home_y) <= self.home_acceptance_radius
                    if success or self.active_navigation_goal or self.navigation_cancel_pending:
                        break
        except Exception as exc:
            reason = "return_home_exception"
            self.get_logger().error(f"Return supervisor failed: {exc}")
            if self.active_navigation_goal:
                self.cancel_active_navigation_and_wait(reason)
        finally:
            if self.navigation_submission_uncertain or self.active_navigation_goal or self.navigation_cancel_pending:
                self.drain_navigation_before_shutdown("return-home finalization")
            self.stop_robot()
            telemetry = self.final_home_telemetry()
            if success and (telemetry["home_error_m"] is None or telemetry["home_error_m"] > self.home_acceptance_radius):
                success = False
                reason = "final_home_localization_invalid" if telemetry["home_error_m"] is None else "final_home_outside_tolerance"
            self.eval_emit("return_home_end", returned_home=bool(success), reason=reason,
                           home_goal_status=home_goal_status, home_reason=reason,
                           last_navigation_terminal_reason=home_terminal_reason,
                           home_goal_stage_index=home_goal_stage_index,
                           **telemetry,
                           return_home_time_s=self.eval_sim_time()-start,
                           navigation_time_s=self.eval_sim_time()-start,
                           planned_path_length_m=planned,
                           attempt_count=attempts, replan_count=replans,
                           goal_status=home_goal_status, goal_result=home_goal_result,
                           diagnostics=self.navigation_diagnostic_snapshot(),
                           pre_home_recovery_used=self.pre_home_recovery_used)
            self.return_distance = None
        return bool(success)


def parse_args():
    parser = argparse.ArgumentParser(description="UAV-UGV agricultural mission manager with quantitative evaluation logging")
    parser.add_argument("--run-id", default=os.environ.get("UAV_UGV_RUN_ID", "manual"))
    parser.add_argument("--scenario", default=os.environ.get("UAV_UGV_SCENARIO", "S1"))
    parser.add_argument("--method", default=os.environ.get("UAV_UGV_METHOD", "P"), choices=["B1", "B2", "B3", "P"])
    parser.add_argument("--seed", type=int, default=(int(os.environ["UAV_UGV_SEED"]) if "UAV_UGV_SEED" in os.environ else None))
    parser.add_argument("--log-path", default=os.environ.get("UAV_UGV_LOG_PATH", "/tmp/uav_benchmark/event_log.jsonl"))
    parser.add_argument("--software-commit", default=os.environ.get("UAV_UGV_SOFTWARE_COMMIT", "unknown"))
    return parser.parse_args(rclpy.utilities.remove_ros_args()[1:])


def main():
    args = parse_args()
    rclpy.init()
    manager = AgriMissionManager(
        run_id=args.run_id, scenario=args.scenario, method=args.method,
        seed=args.seed, log_path=args.log_path, software_commit=args.software_commit,
    )

    try:
        initial_pose = manager.make_pose(0.0, 0.0, 1.57)
        manager.setInitialPose(initial_pose)

        manager.get_logger().info(
            f"Evaluation configuration: run_id={args.run_id}, scenario={args.scenario}, "
            f"method={args.method}, seed={args.seed}, log={args.log_path}"
        )
        manager.get_logger().info("Waiting until Nav2 is active...")
        manager.waitUntilNav2Active()
        manager.get_logger().info("Nav2 is active. UGV ready for UAV targets.")

        manager.get_logger().info("Waiting for covariance-checked AMCL and mapped home pose...")
        home_wait_start = time.monotonic()
        home_timeout = 90.0
        pose_refreshes = 0
        startup_amcl_request_attempts = 0
        startup_amcl_last_request_wall = -math.inf

        while rclpy.ok() and not manager.home_saved:
            rclpy.spin_once(manager, timeout_sec=0.1)
            manager.save_home_from_tf_if_available()

            # Startup must be based on an ACCEPTED AMCL fix, not merely on
            # receipt of any /amcl_pose message. TRANSIENT_LOCAL may deliver
            # an old retained pose that is correctly rejected by timestamp
            # validation. Such a rejected message must not suppress the
            # bootstrap request for a fresh stationary AMCL update.
            now_wall = time.monotonic()
            if (
                manager.last_valid_amcl_pose_msg is None
                and startup_amcl_request_attempts < 3
                and now_wall - home_wait_start >= 2.0
                and now_wall - startup_amcl_last_request_wall >= 3.0
            ):
                startup_amcl_request_attempts += 1
                startup_amcl_last_request_wall = now_wall

                manager.get_logger().warn(
                    f"No accepted AMCL startup fix yet; requesting bounded "
                    f"no-motion update "
                    f"{startup_amcl_request_attempts}/3"
                )

                manager.request_localization_update()
                pose_refreshes += 1

            if time.monotonic() - home_wait_start > home_timeout:
                manager.eval_emit("startup_check", reason="home_validation_timeout", diagnostics=manager.navigation_diagnostic_snapshot())
                manager.get_logger().error("No validated home within 90 seconds; mission aborted")
                manager.eval_emit_mission_end(mission_completed=False, returned_home=False)
                return

        manager.eval_mission_started_sim = manager.eval_sim_time()
        manager.eval_mission_started_monotonic = time.monotonic()
        manager.get_logger().info("Waiting for global costmap before mission_start...")
        if not manager.wait_for_global_costmap(timeout_sec=20.0):
            manager.eval_emit(
                "startup_check",
                reason="global_costmap_timeout",
                diagnostics=manager.navigation_diagnostic_snapshot(),
            )
            manager.get_logger().error(
                "Global costmap unavailable; mission aborted before mission_start"
            )
            manager.eval_emit_mission_end(
                mission_completed=False,
                returned_home=False,
            )
            return
        manager.eval_emit(
            "mission_start",
            start_x=float(manager.current_x), start_y=float(manager.current_y),
            home_x=float(manager.home_x), home_y=float(manager.home_y),
            target_count=(manager.expected_uav_target_count if manager.expected_uav_target_count is not None else 0),
            software_commit=args.software_commit,
        )
        manager.get_logger().info(
            f"Home is ready: x={manager.home_x:.2f}, y={manager.home_y:.2f}, yaw={manager.home_yaw:.2f}"
        )

        while rclpy.ok():
            rclpy.spin_once(manager, timeout_sec=0.1)

            if manager.mission_halted:
                manager.drain_navigation_before_shutdown("mission halted")
                manager.eval_manual_intervention = False
                manager.eval_emit_mission_end(
                    mission_completed=False,
                    returned_home=False,
                    manual_intervention=False,
                )
                break

            if manager.current_target is not None or len(manager.target_queue) > 0:
                manager.process_next_target()

            elif manager.start_deferred_retry_pass():
                pass

            elif (
                manager.uav_scan_complete
                and manager.deferred_targets
                and manager.deferred_retry_pass
                >= manager.max_deferred_retry_passes
            ):
                manager.finalize_deferred_targets()

            elif (
                manager.mission_ready_for_return_home()
                and not manager.return_home_attempted
            ):
                manager.return_home_attempted = True
                success = manager.return_home()

                if success:
                    manager.get_logger().info("Returned home successfully")
                else:
                    manager.get_logger().warn("Return home did not fully succeed")

                manager.stop_robot()
                manager.report_mission_summary()
                all_targets_treated = (
                    len(manager.completed_targets) == len(manager.targets_by_id)
                    and len(manager.targets_by_id) > 0
                    and not manager.deferred_targets
                    and not manager.failed_targets
                    and not manager.permanently_unreachable_targets
                )
                manager.eval_emit_mission_end(
                    mission_completed=bool(success and all_targets_treated),
                    returned_home=bool(success),
                    manual_intervention=False,
                )
                break

            time.sleep(0.1)

    except TimeoutError as exc:
        manager.get_logger().error(f"Bounded infrastructure timeout: {exc}")
        manager.stop_robot()
        manager.drain_navigation_before_shutdown(str(exc))
        manager.eval_emit_mission_end(
            mission_completed=False,
            returned_home=False,
            manual_intervention=False,
        )

    except PhysicalClockError as exc:
        manager.get_logger().error(f"Physical-time infrastructure failure: {exc}")
        manager.stop_robot()
        manager.drain_navigation_before_shutdown(str(exc))
        manager.eval_emit_mission_end(
            mission_completed=False, returned_home=False, manual_intervention=False
        )
        raise

    except KeyboardInterrupt:
        manager.eval_manual_intervention = True
        manager.get_logger().warn("Keyboard interrupt received")
        manager.drain_navigation_before_shutdown("keyboard interrupt")
        manager.stop_robot()
        manager.eval_emit_mission_end(
            mission_completed=False, returned_home=False, manual_intervention=True
        )

    finally:
        try:
            manager.drain_navigation_before_shutdown("node shutdown")
        except Exception:
            pass

        try:
            manager.stop_robot()
        except Exception:
            pass

        try:
            manager.destroy_node()
        except Exception:
            pass

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
