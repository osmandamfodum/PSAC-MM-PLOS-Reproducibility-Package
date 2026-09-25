#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import random
import shlex
import signal
import subprocess
import sys
import time
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

RC12 = Path("/home/drosmanalhussein/robot_ws2/benchmark_results/navigation_v2/release_candidate_rc12")
WS = RC12 / "workspace"
PKG_SRC = WS / "src/my_robot_description"
RESULTS_ROOT = RC12 / "paper2_n10"

ROS_SETUP = Path("/opt/ros/humble/setup.bash")
LOCAL_SETUP = WS / "install/local_setup.bash"

NAV2_PARAMS = PKG_SRC / "config/nav2_params.yaml"
MAP_FILE = PKG_SRC / "maps/my_farm_map.yaml"

SCENARIOS = ("S1", "S2", "S3")
METHODS = ("B1", "B2", "B3", "P")
SEEDS = tuple(range(1001, 1011))
SCHEDULE_RANDOM_SEED = 20260920

EXPECTED_MISSION_IMPL_SHA = "4a06a45303e3ca5692aca79d435365db38e78f445cf0702410b408c8b1f6fae7"
EXPECTED_PREFIX = (WS / "install/my_robot_description").resolve()

DEFAULT_RUN_TIMEOUT_S = 1200
DEFAULT_MAX_INFRA_ATTEMPTS = 8

NONCOMPOSED_NAV2_LAUNCHER = r"""#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

nav2_dir = get_package_share_directory("nav2_bringup")
params_file = os.environ["PAPER2_NAV2_PARAMS"]
map_file = os.environ["PAPER2_NAV2_MAP"]

nav2_bringup = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(nav2_dir, "launch", "bringup_launch.py")
    ),
    launch_arguments={
        "map": map_file,
        "use_sim_time": "true",
        "params_file": params_file,
        "autostart": "false",
        "use_composition": "False",
    }.items(),
)

collision_monitor = Node(
    package="nav2_collision_monitor",
    executable="collision_monitor",
    name="collision_monitor",
    output="screen",
    parameters=[params_file],
)

collision_monitor_lifecycle = Node(
    package="nav2_lifecycle_manager",
    executable="lifecycle_manager",
    name="lifecycle_manager_collision_monitor",
    output="screen",
    parameters=[{
        "use_sim_time": True,
        "autostart": False,
        "node_names": ["collision_monitor"],
    }],
)

ld = LaunchDescription([
    nav2_bringup,
    collision_monitor,
    collision_monitor_lifecycle,
])

ls = LaunchService()
ls.include_launch_description(ld)
raise SystemExit(ls.run())
"""

def write_noncomposed_nav2_launcher(path: Path) -> None:
    path.write_text(NONCOMPOSED_NAV2_LAUNCHER)
    path.chmod(0o755)

LIFECYCLE_STARTUP_HELPER = r"""#!/usr/bin/env python3
import sys
import time

import rclpy
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import ChangeState, GetState
from nav2_msgs.srv import ManageLifecycleNodes


LOCALIZATION_NODES = [
    "map_server",
    "amcl",
]

NAVIGATION_NODES = [
    "controller_server",
    "smoother_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
    "waypoint_follower",
    "velocity_smoother",
]

COLLISION_NODES = [
    "collision_monitor",
]


def fail(msg):
    print(f"[LIFECYCLE][ERROR] {msg}", flush=True)
    raise SystemExit(2)


def wait_service(node, srv_type, name, deadline):
    client = node.create_client(srv_type, name)
    while rclpy.ok():
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            fail(f"timeout waiting for service {name}")
        if client.wait_for_service(timeout_sec=min(1.0, remaining)):
            print(f"[LIFECYCLE] ready: {name}", flush=True)
            return client


def wait_managed_node_services(node, node_names, timeout_s=90.0):
    deadline = time.monotonic() + timeout_s
    for managed in node_names:
        wait_service(node, GetState, f"/{managed}/get_state", deadline)
        wait_service(node, ChangeState, f"/{managed}/change_state", deadline)


def call_startup(node, manager_name, timeout_s=120.0):
    srv_name = f"/{manager_name}/manage_nodes"
    deadline = time.monotonic() + timeout_s
    client = wait_service(node, ManageLifecycleNodes, srv_name, deadline)

    request = ManageLifecycleNodes.Request()
    request.command = ManageLifecycleNodes.Request.STARTUP

    print(f"[LIFECYCLE] STARTUP -> {manager_name}", flush=True)
    future = client.call_async(request)
    remaining = max(0.1, deadline - time.monotonic())
    rclpy.spin_until_future_complete(node, future, timeout_sec=remaining)

    if not future.done():
        fail(f"{manager_name} STARTUP service timed out")

    try:
        response = future.result()
    except Exception as exc:
        fail(f"{manager_name} STARTUP raised: {exc}")

    if response is None or not response.success:
        fail(f"{manager_name} STARTUP returned success=false")

    print(f"[LIFECYCLE] STARTUP success: {manager_name}", flush=True)


def wait_active(node, node_names, timeout_s=90.0):
    deadline = time.monotonic() + timeout_s

    for managed in node_names:
        client = node.create_client(GetState, f"/{managed}/get_state")
        while rclpy.ok():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                fail(f"timeout waiting for {managed} to become active")

            if not client.wait_for_service(timeout_sec=min(1.0, remaining)):
                continue

            future = client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(
                node, future, timeout_sec=min(3.0, remaining)
            )
            if not future.done():
                continue

            try:
                response = future.result()
            except Exception:
                time.sleep(0.2)
                continue

            if response.current_state.id == State.PRIMARY_STATE_ACTIVE:
                print(f"[LIFECYCLE] active: {managed}", flush=True)
                break

            time.sleep(0.2)


def main():
    rclpy.init()
    node = rclpy.create_node("paper2_ordered_lifecycle_startup")

    try:
        # Critical race fix:
        # do not ask a lifecycle manager to transition a node until BOTH
        # get_state and change_state services for every managed node exist.
        wait_managed_node_services(node, LOCALIZATION_NODES)
        call_startup(node, "lifecycle_manager_localization")
        wait_active(node, LOCALIZATION_NODES)

        wait_managed_node_services(node, NAVIGATION_NODES)
        call_startup(node, "lifecycle_manager_navigation")
        wait_active(node, NAVIGATION_NODES)

        wait_managed_node_services(node, COLLISION_NODES)
        call_startup(node, "lifecycle_manager_collision_monitor")
        wait_active(node, COLLISION_NODES)

        print("[LIFECYCLE] ORDERED STARTUP COMPLETE", flush=True)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
"""


def write_lifecycle_startup_helper(path: Path) -> None:
    path.write_text(LIFECYCLE_STARTUP_HELPER)
    path.chmod(0o755)

@dataclass
class ManagedProcess:
    name: str
    proc: subprocess.Popen
    log_handle: object
    log_path: Path

def clean_env(extra: Optional[dict[str, str]] = None) -> dict[str, str]:
    env = {
        "HOME": os.environ.get("HOME", str(Path.home())),
        "USER": os.environ.get("USER", ""),
        "PATH": "/usr/bin:/bin:/usr/sbin:/sbin",
        "TERM": os.environ.get("TERM", "xterm"),
        "PYTHONUNBUFFERED": "1",
    }
    if extra:
        env.update({k: str(v) for k, v in extra.items()})
    return env

def shell_prefix() -> str:
    return (
        "set +u; "
        f"source {shlex.quote(str(ROS_SETUP))}; "
        f"source {shlex.quote(str(LOCAL_SETUP))}; "
    )

def run_ros_shell(command: str, extra_env: Optional[dict[str, str]] = None,
                  timeout: int = 20, check: bool = False) -> subprocess.CompletedProcess:
    return subprocess.run(
        ["/bin/bash", "--noprofile", "--norc", "-c", shell_prefix() + command],
        env=clean_env(extra_env),
        text=True,
        capture_output=True,
        timeout=timeout,
        check=check,
    )

def launch_process(name: str, command: str, log_path: Path,
                   extra_env: dict[str, str]) -> ManagedProcess:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    fh = open(log_path, "ab", buffering=0)
    proc = subprocess.Popen(
        ["/bin/bash", "--noprofile", "--norc", "-c", shell_prefix() + "exec " + command],
        env=clean_env(extra_env),
        stdout=fh,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    return ManagedProcess(name=name, proc=proc, log_handle=fh, log_path=log_path)

def terminate_process(mp: ManagedProcess, grace_s: float = 8.0) -> None:
    if mp.proc.poll() is None:
        try:
            os.killpg(mp.proc.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        deadline = time.monotonic() + grace_s
        while mp.proc.poll() is None and time.monotonic() < deadline:
            time.sleep(0.2)
    if mp.proc.poll() is None:
        try:
            os.killpg(mp.proc.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        deadline = time.monotonic() + 3.0
        while mp.proc.poll() is None and time.monotonic() < deadline:
            time.sleep(0.2)
    if mp.proc.poll() is None:
        try:
            os.killpg(mp.proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        try:
            mp.proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            pass
    try:
        mp.log_handle.close()
    except Exception:
        pass

def cleanup_all(processes: list[ManagedProcess]) -> None:
    for mp in reversed(processes):
        terminate_process(mp)

def log_contains(path: Path, token: str) -> bool:
    try:
        return token in path.read_text(errors="replace")
    except FileNotFoundError:
        return False

def wait_for_log(mp: ManagedProcess, token: str, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if log_contains(mp.log_path, token):
            return
        rc = mp.proc.poll()
        if rc is not None:
            raise RuntimeError(
                f"{mp.name} exited before readiness token {token!r}; rc={rc}; log={mp.log_path}"
            )
        time.sleep(0.5)
    raise RuntimeError(f"Timeout waiting for {mp.name} token {token!r}; log={mp.log_path}")

def _tail_text(path: Path, max_lines: int = 80) -> str:
    try:
        return "\n".join(path.read_text(errors="replace").splitlines()[-max_lines:])
    except FileNotFoundError:
        return "<log missing>"

def wait_for_nav2_active(nav2: ManagedProcess, timeout_s: float = 180.0) -> None:
    """Wait for the *navigation* lifecycle manager, not collision_monitor.

    The previous runner repeatedly invoked `ros2 lifecycle get /bt_navigator`.
    For long batch runs this adds extra ROS CLI discovery processes and can return
    an empty response while Nav2 is still composing.  The Nav2 lifecycle manager's
    own log is the authoritative startup evidence used here.
    """
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if nav2.proc.poll() is not None:
            raise RuntimeError(
                f"nav2 exited before activation; rc={nav2.proc.returncode}; "
                f"tail=\n{_tail_text(nav2.log_path)}"
            )
        try:
            lines = nav2.log_path.read_text(errors="replace").splitlines()
        except FileNotFoundError:
            lines = []
        for line in lines[-250:]:
            low = line.lower()
            if "lifecycle_manager_navigation" in low and "managed nodes are active" in low:
                return
        time.sleep(0.5)
    raise RuntimeError(
        "Nav2 navigation lifecycle manager did not become active within "
        f"{timeout_s:.0f}s. Last log lines:\n{_tail_text(nav2.log_path)}"
    )

def current_ancestor_pids() -> set[int]:
    result: set[int] = set()
    pid = os.getpid()
    while pid > 1 and pid not in result:
        result.add(pid)
        status = Path(f"/proc/{pid}/status")
        try:
            lines = status.read_text().splitlines()
        except (FileNotFoundError, PermissionError):
            break
        ppid = 0
        for line in lines:
            if line.startswith("PPid:"):
                ppid = int(line.split()[1])
                break
        if ppid <= 1 or ppid == pid:
            break
        pid = ppid
    return result

def find_leftovers() -> list[str]:
    excluded = current_ancestor_pids()
    script_names = {"mission_manager.py", "uav_farm_scanner.py", "dynamic_obstacle_controller.py"}
    native_names = {
        "gzserver", "gzclient", "rviz2", "component_container_isolated",
        "collision_monitor", "map_server", "amcl", "controller_server",
        "smoother_server", "planner_server", "behavior_server", "bt_navigator",
        "waypoint_follower", "velocity_smoother", "lifecycle_manager",
    }
    found: list[str] = []
    proc_root = Path("/proc")
    for p in proc_root.iterdir():
        if not p.name.isdigit():
            continue
        pid = int(p.name)
        if pid in excluded:
            continue
        try:
            comm = (p / "comm").read_text().strip()
            argv = [
                x.decode(errors="replace")
                for x in (p / "cmdline").read_bytes().split(b"\0")
                if x
            ]
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
        matched = comm in native_names
        if not matched:
            matched = any(Path(arg).name in native_names for arg in argv)
        if not matched:
            matched = any(Path(arg).name in script_names for arg in argv)
        if matched:
            found.append(f"PID={pid} COMM={comm} CMD={' '.join(argv)}")
    return found

def wait_no_leftovers(timeout_s: float = 30.0) -> list[str]:
    deadline = time.monotonic() + timeout_s
    last: list[str] = []
    while time.monotonic() < deadline:
        last = find_leftovers()
        if not last:
            return []
        time.sleep(1.0)
    return last


def attempt_token_pids(token: str) -> list[int]:
    """Return only processes that inherited this benchmark attempt token."""
    wanted = f"PAPER2_ATTEMPT_TOKEN={token}".encode()
    excluded = current_ancestor_pids()
    found: list[int] = []

    for p in Path("/proc").iterdir():
        if not p.name.isdigit():
            continue
        pid = int(p.name)
        if pid in excluded:
            continue
        try:
            env_items = (p / "environ").read_bytes().split(b"\0")
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
        if wanted in env_items:
            found.append(pid)

    return sorted(found)


def wait_no_attempt_token_pids(token: str, timeout_s: float) -> list[int]:
    deadline = time.monotonic() + timeout_s
    remaining: list[int] = []
    while time.monotonic() < deadline:
        remaining = attempt_token_pids(token)
        if not remaining:
            return []
        time.sleep(0.25)
    return remaining


def signal_pids(pids: list[int], sig: int) -> None:
    for pid in pids:
        try:
            os.kill(pid, sig)
        except ProcessLookupError:
            pass
        except PermissionError:
            pass


def force_cleanup_attempt(token: str) -> list[int]:
    """Escalating cleanup for children that escaped ros2 launch process groups."""
    pids = attempt_token_pids(token)
    if not pids:
        return []

    signal_pids(pids, signal.SIGINT)
    remaining = wait_no_attempt_token_pids(token, 5.0)
    if not remaining:
        return []

    signal_pids(remaining, signal.SIGTERM)
    remaining = wait_no_attempt_token_pids(token, 3.0)
    if not remaining:
        return []

    signal_pids(remaining, signal.SIGKILL)
    return wait_no_attempt_token_pids(token, 2.0)


def cleanup_user_fastdds_shm_if_experiment_idle() -> list[str]:
    """Remove stale Fast DDS SHM only when no experiment ROS processes remain."""
    if find_leftovers():
        return []

    removed: list[str] = []
    shm = Path("/dev/shm")
    uid = os.getuid()

    patterns = ("fastrtps_*", "sem.fastrtps_*")
    for pattern in patterns:
        for p in shm.glob(pattern):
            try:
                if p.stat().st_uid != uid:
                    continue
                p.unlink()
                removed.append(str(p))
            except (FileNotFoundError, PermissionError, OSError):
                pass

    return removed

def verify_cpu_governor() -> None:
    files = sorted(Path("/sys/devices/system/cpu").glob("cpu[0-9]*/cpufreq/scaling_governor"))
    if not files:
        raise RuntimeError("CPU governor information is unavailable; refusing quantitative batch run.")
    governors = set()
    for p in files:
        try:
            governors.add(p.read_text().strip())
        except OSError:
            pass
    if governors != {"performance"}:
        raise RuntimeError(f"CPU governor must be uniformly performance; observed={sorted(governors)}")

def verify_rc12_environment() -> None:
    cp = run_ros_shell("ros2 pkg prefix my_robot_description", timeout=20)
    prefix = cp.stdout.strip()
    if cp.returncode != 0:
        raise RuntimeError(f"ros2 pkg prefix failed: {cp.stderr.strip()}")
    if Path(prefix).resolve() != EXPECTED_PREFIX:
        raise RuntimeError(f"Package contamination: expected {EXPECTED_PREFIX}, got {prefix}")

    impl = PKG_SRC / "scripts/agri_mission/mission_manager.py"
    actual = hashlib.sha256(impl.read_bytes()).hexdigest()
    if actual != EXPECTED_MISSION_IMPL_SHA:
        raise RuntimeError(
            "Accepted RC12 mission_manager hash changed: "
            f"expected={EXPECTED_MISSION_IMPL_SHA}, actual={actual}"
        )

    critical = [
        "mission_manager.py",
        "uav_farm_scanner.py",
        "dynamic_obstacle_controller.py",
    ]
    for name in critical:
        p = WS / "install/my_robot_description/lib/my_robot_description" / name
        if not p.exists() or not os.access(p, os.X_OK):
            raise RuntimeError(f"Critical runtime executable missing/non-executable: {p}")

def package_fingerprint() -> str:
    h = hashlib.sha256()
    for p in sorted(PKG_SRC.rglob("*")):
        rel = p.relative_to(PKG_SRC).as_posix()
        if p.is_symlink():
            h.update(f"L\0{rel}\0{os.readlink(p)}\n".encode())
        elif p.is_file():
            mode = p.stat().st_mode & 0o777
            h.update(f"F\0{rel}\0{mode:o}\0".encode())
            with open(p, "rb") as f:
                for chunk in iter(lambda: f.read(1024 * 1024), b""):
                    h.update(chunk)
            h.update(b"\n")
    return h.hexdigest()

def ensure_fingerprint(root: Path) -> str:
    current = package_fingerprint()
    fp_file = root / "ACCEPTED_CODE_FINGERPRINT.sha256"
    if fp_file.exists():
        expected = fp_file.read_text().strip().split()[0]
        if current != expected:
            raise RuntimeError(
                "Package source fingerprint changed after batch initialization.\n"
                f"expected={expected}\nactual={current}"
            )
    else:
        fp_file.write_text(f"{current}  src/my_robot_description\n")
    return current

def build_schedule() -> list[dict[str, object]]:
    rng = random.Random(SCHEDULE_RANDOM_SEED)
    rows: list[dict[str, object]] = []
    order = 0
    for scenario in SCENARIOS:
        for seed in SEEDS:
            method_order = list(METHODS)
            rng.shuffle(method_order)
            for method in method_order:
                order += 1
                rows.append({
                    "order": order,
                    "scenario": scenario,
                    "method": method,
                    "seed": seed,
                    "run_id": f"P2_{scenario}_{method}_seed{seed}",
                })
    assert len(rows) == 120
    return rows

def ensure_schedule(root: Path) -> list[dict[str, object]]:
    schedule_path = root / "schedule.csv"
    expected = build_schedule()
    if not schedule_path.exists():
        with open(schedule_path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=["order", "scenario", "method", "seed", "run_id"])
            w.writeheader()
            w.writerows(expected)
        return expected

    rows: list[dict[str, object]] = []
    with open(schedule_path, newline="") as f:
        for row in csv.DictReader(f):
            rows.append({
                "order": int(row["order"]),
                "scenario": row["scenario"],
                "method": row["method"],
                "seed": int(row["seed"]),
                "run_id": row["run_id"],
            })
    if rows != expected:
        raise RuntimeError("Existing schedule.csv differs from the fixed Paper2 schedule.")
    return rows

def event_name(obj: dict) -> Optional[str]:
    for key in ("event", "event_type", "name", "type"):
        value = obj.get(key)
        if isinstance(value, str):
            return value
    return None

def parse_events(path: Path) -> tuple[list[dict], Counter]:
    rows: list[dict] = []
    counts: Counter = Counter()
    if not path.exists():
        return rows, counts
    with open(path, errors="replace") as f:
        for lineno, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
            except json.JSONDecodeError as e:
                raise RuntimeError(f"Malformed JSONL at {path}:{lineno}: {e}")
            if not isinstance(obj, dict):
                raise RuntimeError(f"Non-object JSONL entry at {path}:{lineno}")
            rows.append(obj)
            name = event_name(obj)
            if name:
                counts[name] += 1
    return rows, counts

def has_event(path: Path, wanted: str) -> bool:
    if not path.exists():
        return False
    try:
        with open(path, errors="replace") as f:
            for line in f:
                if wanted not in line:
                    continue
                try:
                    obj = json.loads(line)
                    if isinstance(obj, dict) and event_name(obj) == wanted:
                        return True
                except json.JSONDecodeError:
                    pass
                # Fallback supports logger-schema changes while remaining conservative.
                if f'"{wanted}"' in line:
                    return True
    except OSError:
        return False
    return False

def next_attempt_dir(logical_dir: Path) -> tuple[int, Path]:
    attempt = 1
    while (logical_dir / f"attempt_{attempt:02d}").exists():
        attempt += 1
    return attempt, logical_dir / f"attempt_{attempt:02d}"

def write_json(path: Path, obj: dict) -> None:
    path.write_text(json.dumps(obj, indent=2, sort_keys=True) + "\n")

def run_one(row: dict[str, object], attempt_dir: Path, run_timeout_s: int) -> dict:
    scenario = str(row["scenario"])
    method = str(row["method"])
    seed = int(row["seed"])
    run_id = str(row["run_id"])

    attempt_dir.mkdir(parents=True, exist_ok=False)
    logs_dir = attempt_dir / "logs"
    ros_logs = attempt_dir / "ros_logs"
    logs_dir.mkdir()
    ros_logs.mkdir()

    event_log = logs_dir / "event_log.jsonl"
    processes: list[ManagedProcess] = []
    started_wall = time.time()
    started_mono = time.monotonic()

    attempt_token = str(attempt_dir.resolve())

    common_env = {
        "UAV_UGV_RUN_ID": run_id,
        "UAV_UGV_SCENARIO": scenario,
        "UAV_UGV_METHOD": method,
        "UAV_UGV_SEED": str(seed),
        "UAV_UGV_LOG_PATH": str(event_log),
        "ROS_DOMAIN_ID": "0",
        "PAPER2_ATTEMPT_TOKEN": attempt_token,
    }

    metadata = {
        "run_id": run_id,
        "scenario": scenario,
        "method": method,
        "seed": seed,
        "experiment_class": "PAPER2_N10_QUANTITATIVE",
        "workspace": str(WS),
        "mission_manager_sha256": EXPECTED_MISSION_IMPL_SHA,
        "runner_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "nav2_rviz": False,
        "nav2_composition": False,
        "nav2_startup_mode": "non_composed_ordered_manual_lifecycle",
        "runner_cleanup_mode": "attempt_token_force_cleanup_v5",
        "started_unix": started_wall,
    }
    write_json(attempt_dir / "RUN_METADATA.json", metadata)

    def component_env(name: str) -> dict[str, str]:
        env = dict(common_env)
        component_log_dir = ros_logs / name
        component_log_dir.mkdir(parents=True, exist_ok=True)
        env["ROS_LOG_DIR"] = str(component_log_dir)
        return env

    try:
        leftovers = find_leftovers()
        if leftovers:
            raise RuntimeError("Pre-existing experiment processes detected:\n" + "\n".join(leftovers))

        rsp = launch_process(
            "terminal1_rsp",
            "ros2 launch my_robot_description rsp.launch.py",
            logs_dir / "terminal1_rsp.log",
            component_env("terminal1_rsp"),
        )
        processes.append(rsp)
        wait_for_log(rsp, "Successfully spawned entity [agri_robot]", 90)
        # Do not race Nav2 against Gazebo/plugin initialization.  The manual
        # acceptance run waited until odometry was advertised before starting Nav2.
        wait_for_log(rsp, "Advertise odometry on [/odom]", 45)
        time.sleep(2.0)

        # Avoid the ROS 2 composable-node /_container/load_node response timeout
        # observed in repeated batch startups. This launcher reproduces the
        # accepted RC12 Nav2 + collision-monitor stack, but runs Nav2 nodes as
        # separate processes (use_composition=False). RC12 source is untouched.
        nav2_launcher = attempt_dir / "nav2_noncomposed_launcher.py"
        write_noncomposed_nav2_launcher(nav2_launcher)
        nav2_env = component_env("terminal2_nav2")
        nav2_env["PAPER2_NAV2_PARAMS"] = str(NAV2_PARAMS)
        nav2_env["PAPER2_NAV2_MAP"] = str(MAP_FILE)

        nav2 = launch_process(
            "terminal2_nav2",
            f"python3 {shlex.quote(str(nav2_launcher))}",
            logs_dir / "terminal2_nav2.log",
            nav2_env,
        )
        processes.append(nav2)

        lifecycle_helper = attempt_dir / "nav2_ordered_lifecycle_startup.py"
        write_lifecycle_startup_helper(lifecycle_helper)
        lifecycle_log = logs_dir / "nav2_lifecycle_startup.log"

        lifecycle_cp = run_ros_shell(
            f"python3 {shlex.quote(str(lifecycle_helper))}",
            extra_env=nav2_env,
            timeout=330,
        )
        lifecycle_log.write_text(
            lifecycle_cp.stdout
            + ("\n[STDERR]\n" + lifecycle_cp.stderr if lifecycle_cp.stderr else "")
        )

        if lifecycle_cp.returncode != 0:
            raise RuntimeError(
                "Ordered Nav2 lifecycle startup failed; "
                f"rc={lifecycle_cp.returncode}; log={lifecycle_log}; "
                f"tail=\n{_tail_text(lifecycle_log)}"
            )

        if not log_contains(lifecycle_log, "[LIFECYCLE] ORDERED STARTUP COMPLETE"):
            raise RuntimeError(
                "Ordered Nav2 lifecycle helper exited without completion token; "
                f"log={lifecycle_log}; tail=\n{_tail_text(lifecycle_log)}"
            )

        wait_for_nav2_active(nav2, 60)

        mission_cmd = (
            "ros2 run my_robot_description mission_manager.py -- "
            f"--run-id {shlex.quote(run_id)} "
            f"--scenario {shlex.quote(scenario)} "
            f"--method {shlex.quote(method)} "
            f"--seed {seed} "
            f"--log-path {shlex.quote(str(event_log))}"
        )
        mission = launch_process(
            "terminal3_mission",
            mission_cmd,
            logs_dir / "terminal3_mission.log",
            component_env("terminal3_mission"),
        )
        processes.append(mission)
        wait_for_log(mission, "Agri Mission Manager is ready", 60)
        wait_for_log(mission, "Evaluation configuration:", 30)
        time.sleep(2.0)

        dynamic = launch_process(
            "terminal4_dynamic",
            "ros2 run my_robot_description dynamic_obstacle_controller.py",
            logs_dir / "terminal4_dynamic.log",
            component_env("terminal4_dynamic"),
        )
        processes.append(dynamic)
        wait_for_log(dynamic, f"Benchmark scenario mode active: {scenario}", 45)

        scanner = launch_process(
            "terminal5_scanner",
            "ros2 run my_robot_description uav_farm_scanner.py",
            logs_dir / "terminal5_scanner.log",
            component_env("terminal5_scanner"),
        )
        processes.append(scanner)
        wait_for_log(scanner, f"Scenario target filter active: {scenario}", 45)

        deadline = time.monotonic() + run_timeout_s
        while time.monotonic() < deadline:
            if has_event(event_log, "mission_end"):
                break

            for mp in (rsp, nav2, mission, dynamic):
                rc = mp.proc.poll()
                if rc is not None:
                    raise RuntimeError(
                        f"Critical process {mp.name} exited before mission_end; rc={rc}; log={mp.log_path}"
                    )
            scanner_rc = scanner.proc.poll()
            if scanner_rc not in (None, 0):
                raise RuntimeError(
                    f"Scanner exited abnormally before mission_end; rc={scanner_rc}; log={scanner.log_path}"
                )
            time.sleep(1.0)
        else:
            raise RuntimeError(f"Run timeout after {run_timeout_s}s without mission_end")

        rows, counts = parse_events(event_log)
        if not rows:
            raise RuntimeError("event_log.jsonl is empty")
        if counts.get("mission_end", 0) < 1 and not has_event(event_log, "mission_end"):
            raise RuntimeError("mission_end missing from event log")
        if counts.get("mission_start", 0) < 1 and not has_event(event_log, "mission_start"):
            raise RuntimeError("mission_start missing from event log")

        duration = time.monotonic() - started_mono
        result = {
            "valid": True,
            "classification": "VALID_QUANTITATIVE_RUN",
            "reason": "",
            "run_id": run_id,
            "scenario": scenario,
            "method": method,
            "seed": seed,
            "duration_s": round(duration, 3),
            "event_count": len(rows),
            "treatment_completed": counts.get("treatment_completed", 0),
            "target_failed": counts.get("target_failed", 0),
            "target_deferred": counts.get("target_deferred", 0),
            "collision": counts.get("collision", 0),
            "replan": counts.get("replan", 0),
            "recovery_completed": counts.get("recovery_completed", 0),
            "finished_unix": time.time(),
        }
        return result

    except KeyboardInterrupt:
        raise
    except Exception as e:
        return {
            "valid": False,
            "classification": "INFRA_INVALID",
            "reason": str(e),
            "run_id": run_id,
            "scenario": scenario,
            "method": method,
            "seed": seed,
            "duration_s": round(time.monotonic() - started_mono, 3),
            "event_count": 0,
            "treatment_completed": 0,
            "target_failed": 0,
            "target_deferred": 0,
            "collision": 0,
            "replan": 0,
            "recovery_completed": 0,
            "finished_unix": time.time(),
        }
    finally:
        cleanup_all(processes)

        # ros2 launch children can occasionally survive the parent process group
        # after a lifecycle service timeout.  Target only descendants that
        # inherited this attempt's unique environment token.
        token_leftovers = force_cleanup_attempt(attempt_token)
        if token_leftovers:
            (attempt_dir / "POST_CLEANUP_TOKEN_PIDS.txt").write_text(
                "\n".join(str(pid) for pid in token_leftovers) + "\n"
            )

        leftovers = wait_no_leftovers(10)
        if leftovers:
            (attempt_dir / "POST_CLEANUP_LEFTOVERS.txt").write_text(
                "\n".join(leftovers) + "\n"
            )
        else:
            # If all experiment processes are gone, stale Fast DDS SHM from a
            # forced termination is safe to remove before the next attempt.
            removed = cleanup_user_fastdds_shm_if_experiment_idle()
            if removed:
                (attempt_dir / "POST_CLEANUP_FASTDDS_SHM.txt").write_text(
                    "\n".join(removed) + "\n"
                )

def rebuild_summary(root: Path, schedule: list[dict[str, object]]) -> None:
    rows = []
    for item in schedule:
        logical = root / str(item["scenario"]) / str(item["method"]) / f"seed_{int(item['seed'])}"
        status_file = logical / "status.json"
        if status_file.exists():
            status = json.loads(status_file.read_text())
        else:
            status = {
                "valid": False,
                "classification": "PENDING",
                "reason": "",
                "duration_s": "",
                "event_count": "",
                "treatment_completed": "",
                "target_failed": "",
                "target_deferred": "",
                "collision": "",
                "replan": "",
                "recovery_completed": "",
                "attempt": "",
            }
        rows.append({
            "order": item["order"],
            "run_id": item["run_id"],
            "scenario": item["scenario"],
            "method": item["method"],
            "seed": item["seed"],
            "valid": status.get("valid", False),
            "classification": status.get("classification", ""),
            "attempt": status.get("attempt", ""),
            "duration_s": status.get("duration_s", ""),
            "event_count": status.get("event_count", ""),
            "treatment_completed": status.get("treatment_completed", ""),
            "target_failed": status.get("target_failed", ""),
            "target_deferred": status.get("target_deferred", ""),
            "collision": status.get("collision", ""),
            "replan": status.get("replan", ""),
            "recovery_completed": status.get("recovery_completed", ""),
            "reason": status.get("reason", ""),
        })
    summary = root / "summary.csv"
    with open(summary, "w", newline="") as f:
        fieldnames = list(rows[0].keys())
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        w.writerows(rows)

def select_rows(schedule: list[dict[str, object]], only: Optional[str]) -> list[dict[str, object]]:
    if not only:
        return schedule

    parts = [x.strip() for x in only.split(",")]
    if len(parts) != 3 or any(not x for x in parts):
        raise ValueError(
            "--only must be exactly SCENARIO,METHOD,SEED, e.g. --only S3,P,1001"
        )

    scenario, method, seed_text = parts
    scenario = scenario.upper()
    method = method.upper()

    if scenario not in SCENARIOS:
        raise ValueError(f"Unknown scenario {scenario!r}; expected one of {SCENARIOS}")
    if method not in METHODS:
        raise ValueError(f"Unknown method {method!r}; expected one of {METHODS}")
    try:
        seed = int(seed_text)
    except ValueError as exc:
        raise ValueError(f"Invalid seed {seed_text!r}") from exc

    return [
        row for row in schedule
        if str(row["scenario"]) == scenario
        and str(row["method"]) == method
        and int(row["seed"]) == seed
    ]

def main() -> int:
    parser = argparse.ArgumentParser(description="Paper 2 automated 3x4x10 ROS2 benchmark runner")
    parser.add_argument("--dry-run", action="store_true", help="Create/verify schedule and print pending runs only")
    parser.add_argument("--only", help="Select exactly one run: SCENARIO,METHOD,SEED (e.g. S1,B1,1001)")
    parser.add_argument("--run-timeout", type=int, default=DEFAULT_RUN_TIMEOUT_S)
    parser.add_argument("--max-infra-attempts", type=int, default=DEFAULT_MAX_INFRA_ATTEMPTS)
    args = parser.parse_args()

    RESULTS_ROOT.mkdir(parents=True, exist_ok=True)

    print("=" * 78)
    print("PAPER 2 N=10 AUTOMATED BENCHMARK")
    print("=" * 78)
    verify_cpu_governor()
    verify_rc12_environment()
    fp = ensure_fingerprint(RESULTS_ROOT)
    schedule = ensure_schedule(RESULTS_ROOT)
    selected = select_rows(schedule, args.only)
    print(f"Accepted mission SHA : {EXPECTED_MISSION_IMPL_SHA}")
    print(f"Package fingerprint  : {fp}")
    print(f"Planned full runs    : {len(schedule)}")
    print(f"Selected this launch : {len(selected)}")
    print(f"Results root         : {RESULTS_ROOT}")
    print("=" * 78)

    leftovers = find_leftovers()
    if leftovers:
        print("ERROR: experiment processes already running:", file=sys.stderr)
        print("\n".join(leftovers), file=sys.stderr)
        return 2

    if args.dry_run:
        pending = 0
        for row in selected:
            logical = RESULTS_ROOT / str(row["scenario"]) / str(row["method"]) / f"seed_{int(row['seed'])}"
            status = logical / "status.json"
            if status.exists():
                obj = json.loads(status.read_text())
                if obj.get("valid") is True:
                    continue
            pending += 1
            print(f"{int(row['order']):03d}  {row['run_id']}")
        print(f"Pending selected runs: {pending}")
        rebuild_summary(RESULTS_ROOT, schedule)
        return 0

    for idx, row in enumerate(selected, 1):
        verify_cpu_governor()
        verify_rc12_environment()
        ensure_fingerprint(RESULTS_ROOT)

        logical = RESULTS_ROOT / str(row["scenario"]) / str(row["method"]) / f"seed_{int(row['seed'])}"
        logical.mkdir(parents=True, exist_ok=True)
        status_file = logical / "status.json"

        if status_file.exists():
            status = json.loads(status_file.read_text())
            if status.get("valid") is True:
                print(f"[SKIP {idx}/{len(selected)}] {row['run_id']} already VALID")
                continue

        existing_attempts = len(list(logical.glob("attempt_*")))
        if existing_attempts >= args.max_infra_attempts:
            print(
                f"[STOP] {row['run_id']} already has {existing_attempts} invalid attempts; "
                "manual investigation required."
            )
            rebuild_summary(RESULTS_ROOT, schedule)
            return 3

        while True:
            attempt_no, attempt_dir = next_attempt_dir(logical)
            if attempt_no > args.max_infra_attempts:
                print(f"[STOP] {row['run_id']} exceeded infrastructure retry limit.")
                rebuild_summary(RESULTS_ROOT, schedule)
                return 3

            print()
            print("=" * 78)
            print(
                f"[RUN {idx}/{len(selected)} | global {int(row['order'])}/120] "
                f"{row['run_id']} attempt={attempt_no}"
            )
            print("=" * 78)

            result = run_one(row, attempt_dir, args.run_timeout)
            result["attempt"] = attempt_no
            result["attempt_dir"] = str(attempt_dir)
            write_json(attempt_dir / "ATTEMPT_RESULT.json", result)
            write_json(status_file, result)
            rebuild_summary(RESULTS_ROOT, schedule)

            if result["valid"]:
                print(
                    f"[VALID] {row['run_id']} duration={result['duration_s']}s "
                    f"treated={result['treatment_completed']} failed={result['target_failed']}"
                )
                break

            print(f"[INFRA_INVALID] {row['run_id']}: {result['reason']}")
            if attempt_no >= args.max_infra_attempts:
                print("[STOP] Infrastructure retry limit reached. Investigate before continuing.")
                return 3
            print("Retrying the same logical run after clean restart...")
            # Give ROS 2/DDS discovery and Gazebo transport time to fully retire
            # participants before creating the next attempt.
            time.sleep(15)

    rebuild_summary(RESULTS_ROOT, schedule)
    print()
    print("=" * 78)
    print("SELECTED BATCH COMPLETE")
    print(f"Summary: {RESULTS_ROOT / 'summary.csv'}")
    print("=" * 78)
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
