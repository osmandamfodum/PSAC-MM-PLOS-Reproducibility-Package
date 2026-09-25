#!/usr/bin/env python3
"""Quantitative benchmark analysis for the UAV-UGV Gazebo navigation study.

The preferred input is the structured JSONL produced by ros2_evaluation_logger.py.
CSV files are also supported for hand inspection / archival compatibility.

No missing value is fabricated. Derived metrics are computed only when the
required timestamps/fields are present.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Iterable

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

METHOD_ORDER = ["B1", "B2", "B3", "P"]
METHOD_LABELS = {
    "B1": "Direct-to-target",
    "B2": "Single-candidate",
    "B3": "Sequential / No Persistent Target Management",
    "P": "Proposed",
}


def numeric(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return np.nan


def safe_read_csv(path: str, required: Iterable[str]) -> pd.DataFrame:
    df = pd.read_csv(path)
    for col in required:
        if col not in df.columns:
            raise ValueError(f"{path}: missing required column '{col}'")
    return df


def as_bool(series: pd.Series) -> pd.Series:
    if series.dtype == bool:
        return series
    return pd.to_numeric(series, errors="coerce").eq(1)


def mean_sd(values: pd.Series) -> tuple[float, float, int]:
    x = pd.to_numeric(values, errors="coerce").dropna().to_numpy(dtype=float)
    n = len(x)
    if n == 0:
        return np.nan, np.nan, 0
    return float(np.mean(x)), float(np.std(x, ddof=1)) if n > 1 else 0.0, n


def load_events(path: str) -> list[dict]:
    rows = []
    with open(path, "r", encoding="utf-8") as f:
        for line_no, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(f"Invalid JSONL at line {line_no}: {exc}") from exc
            if "event" not in obj or "run_id" not in obj:
                raise ValueError(f"JSONL line {line_no} must contain event and run_id")
            rows.append(obj)
    return rows


def derive_from_events(events: list[dict]):
    runs = {}
    targets = {}
    candidates = []

    def run(rec):
        return runs.setdefault(rec["run_id"], {
            "run_id": rec["run_id"], "scenario": rec.get("scenario"), "method": rec.get("method"),
            "seed": rec.get("seed"), "target_count": np.nan, "start_x": np.nan, "start_y": np.nan,
            "home_x": np.nan, "home_y": np.nan, "mission_start_time": np.nan, "mission_end_time": np.nan,
            "mission_completed": np.nan, "returned_home": np.nan, "manual_intervention": 0,
            "collision_count": 0, "software_commit": None,
        })

    for e in events:
        r = run(e)
        et = e["event"]
        t = numeric(e.get("sim_time_s"))
        if et == "mission_start":
            r.update({k: e.get(k, r[k]) for k in ["start_x","start_y","home_x","home_y","target_count","software_commit"]})
            r["mission_start_time"] = t
        elif et == "collision":
            r["collision_count"] += 1
        elif et == "mission_end":
            r["mission_end_time"] = t
            r["mission_completed"] = e.get("mission_completed")
            r["returned_home"] = e.get("returned_home")
            r["manual_intervention"] = e.get("manual_intervention", 0)
        elif et in {"return_home_end"}:
            r["returned_home"] = e.get("returned_home")

        target_id = e.get("target_id")
        if target_id is not None:
            key = (e["run_id"], str(target_id))
            tar = targets.setdefault(key, {
                "run_id": e["run_id"], "target_id": str(target_id),
                "target_x": e.get("target_x", np.nan), "target_y": e.get("target_y", np.nan),
                "target_order": e.get("target_order", np.nan), "target_start_time": np.nan,
                "goal_accepted_time": np.nan, "target_reached_time": np.nan,
                "treatment_time": np.nan, "target_completed_time": np.nan,
                "target_success": np.nan, "attempt_count": 0, "replan_count": 0,
                "recovery_count": 0, "planned_path_length_m": np.nan,
                "actual_path_length_m": np.nan, "approach_error_m": np.nan,
                "final_target_distance_m": np.nan, "final_service_distance_m": np.nan,
                "planning_time_s": np.nan, "navigation_time_s": np.nan,
                "total_target_time_s": np.nan, "failure_reason": "",
            })
            if e.get("target_x") is not None: tar["target_x"] = e["target_x"]
            if e.get("target_y") is not None: tar["target_y"] = e["target_y"]
            if e.get("target_order") is not None: tar["target_order"] = e["target_order"]
            if et in {"target_received", "target_queued"} and np.isnan(tar["target_start_time"]):
                tar["target_start_time"] = t
            if et == "goal_accepted":
                tar["goal_accepted_time"] = t
                tar["attempt_count"] = max(tar["attempt_count"], int(e.get("attempt") or 0))
            elif et == "candidate_selected":
                for candidate_row in reversed(candidates):
                    if (
                        candidate_row["run_id"] == e["run_id"]
                        and candidate_row["target_id"] == str(target_id)
                        and candidate_row["attempt"] == e.get("attempt")
                        and candidate_row["candidate_id"] == e.get("candidate_id")
                        and not candidate_row["selected"]
                    ):
                        candidate_row["selected"] = 1
                        break
                tar["planned_path_length_m"] = numeric(e.get("path_length_m"))
                tar["planning_time_s"] = numeric(e.get("planning_time_s"))
                tar["attempt_count"] = max(tar["attempt_count"], int(e.get("attempt") or 0))
            elif et == "candidate_checked":
                candidates.append({
                    "run_id": e["run_id"], "scenario": e.get("scenario"), "method": e.get("method"),
                    "target_id": str(target_id), "attempt": e.get("attempt"),
                    "candidate_id": e.get("candidate_id"), "angle_deg": e.get("angle_deg"),
                    "radius_m": e.get("radius_m"), "x": e.get("x"), "y": e.get("y"),
                    "nav_precheck_safe": e.get("nav_precheck_safe"), "uav_safe": e.get("uav_safe"),
                    "planner_feasible": e.get("planner_feasible"), "path_length_m": e.get("path_length_m"),
                    "planning_time_s": e.get("planning_time_s"), "rejected_reason": e.get("rejected_reason"),
                    "selected": 0,
                })
            elif et == "navigation_progress":
                # Actual path is accumulated later from all progress samples.
                pass
            elif et == "replan":
                tar["replan_count"] += 1
                tar["attempt_count"] = max(tar["attempt_count"], int(e.get("attempt") or 0))
            elif et == "recovery_completed" or et == "recovery_started":
                if et == "recovery_started":
                    tar["recovery_count"] += 1
            elif et == "target_reached":
                tar["target_reached_time"] = t
                tar["final_target_distance_m"] = numeric(e.get("final_target_distance_m"))
                tar["final_service_distance_m"] = numeric(e.get("final_service_distance_m"))
                tar["approach_error_m"] = numeric(e.get("final_target_distance_m"))
                tar["navigation_time_s"] = t - numeric(tar["goal_accepted_time"]) if not np.isnan(numeric(tar["goal_accepted_time"])) else np.nan
            elif et == "treatment_completed":
                tar["treatment_time"] = t
                tar["target_completed_time"] = t
                tar["target_success"] = e.get("target_success", 1)
                start = numeric(tar["target_start_time"])
                tar["total_target_time_s"] = t - start if not np.isnan(start) else np.nan
            elif et == "target_failed":
                tar["target_success"] = 0
                tar["target_completed_time"] = t
                tar["failure_reason"] = e.get("failure_reason", "")
                tar["attempt_count"] = max(tar["attempt_count"], int(e.get("attempt") or 0))
            elif et == "target_deferred":
                tar["failure_reason"] = e.get("deferral_reason", "deferred")
                tar["attempt_count"] = max(tar["attempt_count"], int(e.get("attempt") or 0))

    # Accumulate odometry distance using navigation_progress positions per target.
    by_target = {}
    for e in events:
        if e.get("event") != "navigation_progress" or e.get("target_id") is None:
            continue
        key = (e["run_id"], str(e["target_id"]))
        x, y = numeric(e.get("robot_x")), numeric(e.get("robot_y"))
        if np.isnan(x) or np.isnan(y):
            continue
        by_target.setdefault(key, []).append((numeric(e.get("sim_time_s")), x, y))
    for key, pts in by_target.items():
        pts.sort(key=lambda z: np.nan_to_num(z[0], nan=np.inf))
        dist = 0.0
        valid = 0
        for a, b in zip(pts, pts[1:]):
            if np.isnan(a[1]) or np.isnan(a[2]) or np.isnan(b[1]) or np.isnan(b[2]):
                continue
            dist += math.hypot(b[1]-a[1], b[2]-a[2]); valid += 1
        if key in targets and valid:
            targets[key]["actual_path_length_m"] = dist

    runs_df = pd.DataFrame(list(runs.values()))
    targets_df = pd.DataFrame(list(targets.values()))
    candidates_df = pd.DataFrame(candidates)
    if not runs_df.empty:
        received_counts = {}
        for e in events:
            if e.get("event") == "target_received" and e.get("target_id") is not None:
                received_counts.setdefault(e["run_id"], set()).add(str(e.get("target_id")))
        for run_id, target_ids in received_counts.items():
            mask = runs_df["run_id"].eq(run_id)
            if mask.any():
                current = pd.to_numeric(runs_df.loc[mask, "target_count"], errors="coerce")
                if current.isna().all() or float(current.iloc[0]) <= 0:
                    runs_df.loc[mask, "target_count"] = len(target_ids)
        runs_df["mission_completed"] = runs_df["mission_completed"].map(lambda x: x if isinstance(x,bool) else (np.nan if pd.isna(x) else bool(int(x))))
        runs_df["returned_home"] = runs_df["returned_home"].map(lambda x: x if isinstance(x,bool) else (np.nan if pd.isna(x) else bool(int(x))))
    return runs_df, targets_df, candidates_df


def aggregate_runs(runs: pd.DataFrame, targets: pd.DataFrame) -> pd.DataFrame:
    targets = targets.copy()
    targets["target_success"] = as_bool(targets["target_success"])
    tgt_agg = targets.groupby("run_id", as_index=False).agg(
        targets_attempted=("target_id", "count"),
        targets_successful=("target_success", "sum"),
        mean_path_length_m=("planned_path_length_m", "mean"),
        mean_actual_path_length_m=("actual_path_length_m", "mean"),
        mean_navigation_time_s=("navigation_time_s", "mean"),
        mean_total_target_time_s=("total_target_time_s", "mean"),
        mean_approach_error_m=("approach_error_m", "mean"),
        mean_planning_time_s=("planning_time_s", "mean"),
        total_replans=("replan_count", "sum"),
        total_recoveries=("recovery_count", "sum"),
        mean_attempt_count=("attempt_count", "mean"),
    )
    out = runs.merge(tgt_agg, on="run_id", how="left")
    out["target_success_rate_pct"] = np.where(out["targets_attempted"] > 0, 100.0 * out["targets_successful"] / out["targets_attempted"], np.nan)
    return out


def summarize_method_scenario(run_metrics: pd.DataFrame) -> pd.DataFrame:
    rows = []
    for (scenario, method), g in run_metrics.groupby(["scenario", "method"], dropna=False):
        row = {"scenario": scenario, "method": method, "method_label": METHOD_LABELS.get(method, method), "n_runs": len(g)}
        row["mission_completion_pct"] = 100.0 * g["mission_completed"].astype(float).mean() if len(g) else np.nan
        attempted = g["targets_attempted"].sum()
        row["target_success_pct"] = 100.0 * g["targets_successful"].sum() / attempted if attempted else np.nan
        for src, dst in [("mean_path_length_m","path_length_m"),("mean_actual_path_length_m","actual_path_length_m"),("mean_navigation_time_s","navigation_time_s"),("mean_approach_error_m","approach_error_m"),("mean_planning_time_s","planning_time_s"),("total_replans","replans_per_run"),("total_recoveries","recoveries_per_run"),("mean_attempt_count","attempts_per_target")]:
            m, s, n = mean_sd(g[src]); row[f"{dst}_mean"], row[f"{dst}_sd"], row[f"{dst}_n"] = m, s, n
        rows.append(row)
    return pd.DataFrame(rows)


def overall_summary(run_metrics: pd.DataFrame) -> pd.DataFrame:
    rows=[]
    for method, g in run_metrics.groupby("method"):
        attempted=g["targets_attempted"].sum();
        def agg(col):
            return float(g[col].mean()) if len(g) else np.nan
        rows.append({"method":method,"method_label":METHOD_LABELS.get(method,method),"n_runs":len(g),
            "mission_completion_pct":100*g["mission_completed"].astype(float).mean(),
            "target_success_pct":100*g["targets_successful"].sum()/attempted if attempted else np.nan,
            "path_length_mean_m":agg("mean_path_length_m"),"path_length_sd_m":g["mean_path_length_m"].std(ddof=1),
            "navigation_time_mean_s":agg("mean_navigation_time_s"),"navigation_time_sd_s":g["mean_navigation_time_s"].std(ddof=1),
            "approach_error_mean_m":agg("mean_approach_error_m"),"approach_error_sd_m":g["mean_approach_error_m"].std(ddof=1),
            "replans_per_run_mean":agg("total_replans"),"replans_per_run_sd":g["total_replans"].std(ddof=1),
            "recoveries_per_run_mean":agg("total_recoveries"),"recoveries_per_run_sd":g["total_recoveries"].std(ddof=1)})
    return pd.DataFrame(rows)


def candidate_summary(candidates: pd.DataFrame) -> pd.DataFrame:
    if candidates.empty:
        return pd.DataFrame()
    c=candidates.copy()
    for col in ["nav_precheck_safe","uav_safe","planner_feasible","selected"]: c[col]=as_bool(c[col])
    out=c.groupby(["scenario","method"],dropna=False).agg(
        candidates_evaluated=("candidate_id","count"),nav_precheck_safe=("nav_precheck_safe","sum"),uav_safe=("uav_safe","sum"),planner_feasible=("planner_feasible","sum"),selected=("selected","sum"),mean_candidate_path_m=("path_length_m","mean")).reset_index()
    out["candidate_feasibility_pct"]=100*out["planner_feasible"]/out["candidates_evaluated"].replace(0,np.nan)
    return out


def save_bar(data: pd.DataFrame, value_col: str, error_col: str | None, title: str, ylabel: str, path: Path, percent=False):
    methods=[m for m in METHOD_ORDER if m in set(data["method"])]
    d=data.set_index("method").reindex(methods).reset_index()
    x=np.arange(len(d)); fig,ax=plt.subplots(figsize=(8,4.8)); y=pd.to_numeric(d[value_col],errors="coerce").to_numpy()
    err=None if error_col is None else pd.to_numeric(d[error_col],errors="coerce").fillna(0).to_numpy()
    ax.bar(x,y,yerr=err,capsize=4 if error_col else 0); ax.set_xticks(x); ax.set_xticklabels([METHOD_LABELS.get(m,m) for m in d["method"]],rotation=18,ha="right"); ax.set_ylabel(ylabel); ax.set_title(title); ax.grid(axis="y",alpha=0.25)
    if percent: ax.set_ylim(0,100)
    fig.tight_layout(); fig.savefig(path,dpi=300,bbox_inches="tight"); plt.close(fig)


def main():
    ap=argparse.ArgumentParser(description="Analyze UAV-UGV benchmark JSONL or CSV tables")
    ap.add_argument("--events",help="Structured JSONL log; preferred automated input")
    ap.add_argument("--runs"); ap.add_argument("--targets"); ap.add_argument("--candidates")
    ap.add_argument("--out",default="results"); args=ap.parse_args()
    if not args.events and not (args.runs and args.targets and args.candidates):
        ap.error("provide --events OR --runs --targets --candidates")
    out=Path(args.out); out.mkdir(parents=True,exist_ok=True)
    if args.events:
        events=load_events(args.events); runs,targets,candidates=derive_from_events(events)
        runs.to_csv(out/'runs_derived.csv',index=False); targets.to_csv(out/'target_events_derived.csv',index=False); candidates.to_csv(out/'candidate_events_derived.csv',index=False)
        pd.Series([e["event"] for e in events]).value_counts().rename_axis('event').reset_index(name='count').to_csv(out/'event_counts.csv',index=False)
    else:
        runs=safe_read_csv(args.runs,["run_id","scenario","method","mission_completed"]); targets=safe_read_csv(args.targets,["run_id","target_id","target_success"]); candidates=safe_read_csv(args.candidates,["run_id","target_id","candidate_id","planner_feasible"])
    rm=aggregate_runs(runs,targets); rm.to_csv(out/'run_metrics.csv',index=False)
    ms=summarize_method_scenario(rm); ms.to_csv(out/'method_scenario_summary.csv',index=False)
    cs=candidate_summary(candidates); cs.to_csv(out/'candidate_summary.csv',index=False)
    overall=overall_summary(rm); overall.to_csv(out/'overall_summary.csv',index=False)
    save_bar(overall,'mission_completion_pct',None,'Mission completion rate','Mission completion (%)',out/'fig1_mission_completion.png',True)
    save_bar(overall,'target_success_pct',None,'Target success rate','Target success (%)',out/'fig2_target_success.png',True)
    save_bar(overall,'path_length_mean_m','path_length_sd_m','Mean planned path length','Path length (m)',out/'fig3_path_length.png')
    save_bar(overall,'navigation_time_mean_s','navigation_time_sd_s','Mean target navigation time','Navigation time (s)',out/'fig4_navigation_time.png')
    save_bar(overall,'replans_per_run_mean','replans_per_run_sd','Replanning frequency','Replans per mission',out/'fig5_replans.png')
    save_bar(overall,'recoveries_per_run_mean','recoveries_per_run_sd','Recovery frequency','Recovery events per mission',out/'fig6_recoveries.png')
    print(f"Wrote benchmark results to {out.resolve()}")
    print(overall.to_string(index=False))

if __name__=='__main__': main()
