import math
from typing import Callable, Dict, List, Optional, Tuple


TARGET_DISCOVERED = "DISCOVERED"
TARGET_QUEUED = "QUEUED"
TARGET_ACTIVE = "ACTIVE"
TARGET_NAVIGATING = "NAVIGATING"
TARGET_TREATED = "TREATED"
TARGET_DEFERRED = "DEFERRED"
TARGET_UNREACHABLE = "UNREACHABLE"
TARGET_FAILED = "FAILED"


class TargetManager:
    """Owns mission target state without depending on ROS 2 APIs."""

    def __init__(self):
        self.target_queue: List[Dict] = []
        self.current_target: Optional[Dict] = None
        self.completed_targets: List[Dict] = []
        self.deferred_targets: List[Dict] = []
        self.permanently_unreachable_targets: List[Dict] = []
        self.failed_targets: List[Dict] = []
        self.targets_by_id: Dict[str, Dict] = {}

        self.uav_scan_complete = False
        self.expected_uav_target_count: Optional[int] = None
        self.deferred_retry_pass = 0
        self.return_home_attempted = False
        self.mission_summary_reported = False

    @staticmethod
    def make_target_id(x: float, y: float) -> str:
        return f"plant@({x:.3f},{y:.3f})"

    def set_target_state(self, target: Dict, state: str) -> Tuple[str, str]:
        old_state = target.get("state", "NONE")
        target["state"] = state
        return old_state, state

    def is_duplicate_target(self, x: float, y: float) -> bool:
        return self.make_target_id(x, y) in self.targets_by_id

    def add_target(self, x: float, y: float, source: str) -> Tuple[bool, Optional[Dict], str]:
        if not math.isfinite(x) or not math.isfinite(y):
            return False, None, "invalid_coordinates"

        target_id = self.make_target_id(x, y)
        if self.is_duplicate_target(x, y):
            return False, self.targets_by_id[target_id], "duplicate"

        target = {
            "id": target_id,
            "x": float(x),
            "y": float(y),
            "source": source,
            "state": TARGET_DISCOVERED,
            "candidate_cycles": 0,
            "no_candidate_cycles": 0,
            "deferred_retries": 0,
            "infrastructure_retries": 0,
            "failure_reason": "",
            "failure_history": [],
        }
        self.targets_by_id[target_id] = target
        self.target_queue.append(target)
        return True, target, "queued"

    def mark_scan_complete(self, expected_target_count: int):
        self.expected_uav_target_count = int(expected_target_count)
        self.uav_scan_complete = True

    def select_nearest_target(
        self,
        distance_fn: Callable[[float, float], float],
    ) -> Optional[Dict]:
        if self.current_target is not None:
            return self.current_target

        if not self.target_queue:
            return None

        nearest_index = 0
        nearest_distance = float("inf")

        for index, target in enumerate(self.target_queue):
            distance = distance_fn(target["x"], target["y"])
            if distance < nearest_distance:
                nearest_distance = distance
                nearest_index = index

        self.current_target = self.target_queue.pop(nearest_index)
        return self.current_target

    def clear_current_target(self):
        self.current_target = None

    def complete_current_target(self, target: Dict):
        self.set_target_state(target, TARGET_TREATED)
        self.completed_targets.append(target)
        self.clear_current_target()

    def fail_current_target(self, target: Dict):
        self.set_target_state(target, TARGET_FAILED)
        self.failed_targets.append(target)
        self.clear_current_target()

    def start_deferred_retry_pass(
        self,
        max_deferred_retry_passes: int,
    ) -> Tuple[bool, List[Dict]]:
        if (
            self.current_target is not None
            or self.target_queue
            or not self.deferred_targets
            or not self.uav_scan_complete
            or self.deferred_retry_pass >= max_deferred_retry_passes
        ):
            return False, []

        self.deferred_retry_pass += 1
        retry_targets = list(self.deferred_targets)
        self.deferred_targets.clear()

        for target in retry_targets:
            target["deferred_retries"] += 1
            self.target_queue.append(target)

        return True, retry_targets

    def finalize_deferred_targets(self, max_deferred_retry_passes: int) -> List[Dict]:
        if self.deferred_retry_pass < max_deferred_retry_passes:
            return []

        finalized = list(self.deferred_targets)
        for target in finalized:
            self.set_target_state(target, TARGET_UNREACHABLE)
            self.permanently_unreachable_targets.append(target)

        self.deferred_targets.clear()
        return finalized

    def mission_targets_terminal(self) -> bool:
        terminal_states = {
            TARGET_TREATED,
            TARGET_UNREACHABLE,
            TARGET_FAILED,
        }
        return all(
            target["state"] in terminal_states
            for target in self.targets_by_id.values()
        )

    def mission_ready_for_return_home(
        self,
        active_navigation_goal: bool,
        manual_recovery_active: bool,
    ) -> bool:
        if not self.uav_scan_complete:
            return False
        if self.expected_uav_target_count is None:
            return False
        if len(self.targets_by_id) < self.expected_uav_target_count:
            return False
        if self.target_queue or self.deferred_targets:
            return False
        if self.current_target is not None:
            return False
        if active_navigation_goal or manual_recovery_active:
            return False

        return self.mission_targets_terminal()

    def untreated_targets(self) -> List[Dict]:
        return self.permanently_unreachable_targets + self.failed_targets
