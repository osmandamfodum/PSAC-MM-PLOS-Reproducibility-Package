#!/usr/bin/env python3
"""Structured evaluation logger for ROS 2 mission_manager.py.

Each call writes one JSON object per line. Use a monotonic/simulation timestamp
for timing metrics and keep wall-clock time only for auditability.
"""
from __future__ import annotations

import json
from pathlib import Path
import time
from typing import Any


class EvaluationLogger:
    EVENTS = {
        "mission_start",
        "localization_check",
        "startup_check", "return_home_start_check",
        "target_received",
        "target_queued",
        "target_selected",
        "candidate_checked",
        "candidate_selected",
        "goal_accepted",
        "goal_rejected",
        "navigation_progress",
        "stall_detected",
        "cancel_requested",
        "cancel_terminal",
        "recovery_started",
        "recovery_completed",
        "replan",
        "replan_suppressed",
        "obstacle_detected",
        "target_reached",
        "treatment_completed",
        "target_failed",
        "target_deferred",
        "mission_end",
        "collision",
        "dynamic_obstacle_wait",
        "dynamic_obstacle_cleared",
        "dynamic_obstacle_persistent",
        "recovery_suppressed",
        "return_home_plan",
        "return_home_stage",
        "return_home_start",
        "return_home_end",
        "infrastructure_fault",
    }

    def __init__(self, path: str | Path, run_id: str, scenario: str, method: str, seed: int | None = None):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.run_id = run_id
        self.scenario = scenario
        self.method = method
        self.seed = seed

    def emit(self, event: str, sim_time_s: float | None = None, **fields: Any) -> None:
        if event not in self.EVENTS:
            raise ValueError(f"Unsupported evaluation event: {event}")
        record = {
            "event": event,
            "run_id": self.run_id,
            "scenario": self.scenario,
            "method": self.method,
            "seed": self.seed,
            "wall_time_unix": time.time(),
            "sim_time_s": sim_time_s,
        }
        record.update(fields)
        with self.path.open("a", encoding="utf-8") as f:
            f.write(json.dumps(record, separators=(",", ":"), allow_nan=False) + "\n")
