#!/usr/bin/env python3
from pathlib import Path
import math
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

ROOT = Path(
    "/home/drosmanalhussein/robot_ws2/benchmark_results/"
    "navigation_v2/release_candidate_rc12/paper2_analysis"
)
SRC = ROOT / "paper1_style"
OUT = ROOT / "publication"
TABLES = OUT / "tables"
FIGS = OUT / "figures"

for p in (OUT, TABLES, FIGS):
    p.mkdir(parents=True, exist_ok=True)

METHODS = ["B1", "B2", "B3", "P"]
SCENARIOS = ["S1", "S2", "S3"]
LABELS = {
    "B1": "Direct-to-target",
    "B2": "Single-candidate",
    "B3": "No persistent management",
    "P": "Proposed (PSAC-MM)",
}

run_metrics = pd.read_csv(SRC / "run_metrics.csv")
runs = pd.read_csv(SRC / "runs_derived.csv")
candidates = pd.read_csv(SRC / "candidate_events_derived.csv")
comparison = pd.read_csv(ROOT / "paper1_vs_paper2_primary.csv")

# Add return-home and candidate activity to run-level table.
rm = run_metrics.merge(
    runs[["run_id", "returned_home", "collision_count"]],
    on="run_id",
    how="left",
    suffixes=("", "_run"),
)

if not candidates.empty:
    cand = (
        candidates.groupby("run_id", as_index=False)
        .agg(candidate_checks=("candidate_id", "count"))
    )
    rm = rm.merge(cand, on="run_id", how="left")
else:
    rm["candidate_checks"] = 0

rm["candidate_checks"] = rm["candidate_checks"].fillna(0)

def as_bool_num(series):
    def f(v):
        if pd.isna(v):
            return np.nan
        if isinstance(v, bool):
            return 1.0 if v else 0.0
        s = str(v).strip().lower()
        if s in ("true", "1", "1.0"):
            return 1.0
        if s in ("false", "0", "0.0"):
            return 0.0
        return np.nan
    return series.map(f)

rm["mission_completed_num"] = as_bool_num(rm["mission_completed"])
rm["returned_home_num"] = as_bool_num(rm["returned_home"])

# Student-t critical values for 95% two-sided CI; fallback to normal for larger n.
T95 = {
    1: 12.706, 2: 4.303, 3: 3.182, 4: 2.776, 5: 2.571,
    6: 2.447, 7: 2.365, 8: 2.306, 9: 2.262, 10: 2.228,
    11: 2.201, 12: 2.179, 13: 2.160, 14: 2.145, 15: 2.131,
    16: 2.120, 17: 2.110, 18: 2.101, 19: 2.093, 20: 2.086,
    21: 2.080, 22: 2.074, 23: 2.069, 24: 2.064, 25: 2.060,
    26: 2.056, 27: 2.052, 28: 2.048, 29: 2.045, 30: 2.042,
}

def mean_sd_ci(series):
    x = pd.to_numeric(series, errors="coerce").dropna().to_numpy(float)
    n = len(x)
    if n == 0:
        return np.nan, np.nan, np.nan, np.nan, 0
    mean = float(np.mean(x))
    if n == 1:
        return mean, 0.0, mean, mean, 1
    sd = float(np.std(x, ddof=1))
    tcrit = T95.get(n - 1, 1.96)
    half = tcrit * sd / math.sqrt(n)
    return mean, sd, mean - half, mean + half, n

def wilson(successes, total):
    if total <= 0:
        return np.nan, np.nan, np.nan
    z = 1.959963984540054
    p = successes / total
    den = 1 + z*z/total
    center = (p + z*z/(2*total)) / den
    half = z * math.sqrt(p*(1-p)/total + z*z/(4*total*total)) / den
    return 100*p, 100*max(0, center-half), 100*min(1, center+half)

rows = []

for s in SCENARIOS:
    for m in METHODS:
        g = rm[(rm["scenario"] == s) & (rm["method"] == m)].copy()

        attempted = int(pd.to_numeric(g["targets_attempted"], errors="coerce").sum())
        successful = int(pd.to_numeric(g["targets_successful"], errors="coerce").sum())
        ts, ts_lo, ts_hi = wilson(successful, attempted)

        mc_valid = g["mission_completed_num"].dropna()
        mc_succ = int(mc_valid.sum())
        mc, mc_lo, mc_hi = wilson(mc_succ, len(mc_valid))

        rh_valid = g["returned_home_num"].dropna()
        rh_succ = int(rh_valid.sum())
        rh, rh_lo, rh_hi = wilson(rh_succ, len(rh_valid))

        row = {
            "scenario": s,
            "method": m,
            "method_label": LABELS[m],
            "n_runs": len(g),

            "targets_successful": successful,
            "targets_attempted": attempted,
            "target_success_pct": ts,
            "target_success_ci95_low": ts_lo,
            "target_success_ci95_high": ts_hi,

            "missions_completed": mc_succ,
            "mission_completion_pct": mc,
            "mission_completion_ci95_low": mc_lo,
            "mission_completion_ci95_high": mc_hi,

            "returned_home_runs": rh_succ,
            "return_home_pct": rh,
            "return_home_ci95_low": rh_lo,
            "return_home_ci95_high": rh_hi,

            "collision_count_mean": pd.to_numeric(
                g["collision_count"], errors="coerce"
            ).mean(),
        }

        metrics = {
            "actual_path_m": "mean_actual_path_length_m",
            "navigation_time_s": "mean_navigation_time_s",
            "approach_error_m": "mean_approach_error_m",
            "planning_time_s": "mean_planning_time_s",
            "replans_per_run": "total_replans",
            "recoveries_per_run": "total_recoveries",
            "attempts_per_target": "mean_attempt_count",
            "candidate_checks_per_run": "candidate_checks",
        }

        for prefix, col in metrics.items():
            mean, sd, lo, hi, n = mean_sd_ci(g[col])
            row[f"{prefix}_mean"] = mean
            row[f"{prefix}_sd"] = sd
            row[f"{prefix}_ci95_low"] = lo
            row[f"{prefix}_ci95_high"] = hi
            row[f"{prefix}_n"] = n

        rows.append(row)

scenario = pd.DataFrame(rows)
scenario.to_csv(TABLES / "table01_scenario_method_full.csv", index=False)

primary_cols = [
    "scenario", "method", "n_runs",
    "targets_successful", "targets_attempted",
    "target_success_pct", "target_success_ci95_low", "target_success_ci95_high",
    "missions_completed", "mission_completion_pct",
    "mission_completion_ci95_low", "mission_completion_ci95_high",
    "returned_home_runs", "return_home_pct",
    "return_home_ci95_low", "return_home_ci95_high",
    "collision_count_mean",
]
scenario[primary_cols].to_csv(TABLES / "table02_primary_results.csv", index=False)

eff_cols = [
    "scenario", "method", "n_runs",
    "actual_path_m_mean", "actual_path_m_sd",
    "actual_path_m_ci95_low", "actual_path_m_ci95_high",
    "navigation_time_s_mean", "navigation_time_s_sd",
    "navigation_time_s_ci95_low", "navigation_time_s_ci95_high",
    "approach_error_m_mean", "approach_error_m_sd",
    "approach_error_m_ci95_low", "approach_error_m_ci95_high",
]
scenario[eff_cols].to_csv(TABLES / "table03_navigation_efficiency.csv", index=False)

behavior_cols = [
    "scenario", "method", "n_runs",
    "planning_time_s_mean", "planning_time_s_sd",
    "replans_per_run_mean", "replans_per_run_sd",
    "recoveries_per_run_mean", "recoveries_per_run_sd",
    "attempts_per_target_mean", "attempts_per_target_sd",
    "candidate_checks_per_run_mean", "candidate_checks_per_run_sd",
]
scenario[behavior_cols].to_csv(TABLES / "table04_planning_recovery.csv", index=False)

comparison.to_csv(TABLES / "table05_paper1_vs_paper2_primary.csv", index=False)

# Overall Paper 2 summary.
overall_rows = []
for m in METHODS:
    g = rm[rm["method"] == m]

    attempted = int(pd.to_numeric(g["targets_attempted"], errors="coerce").sum())
    successful = int(pd.to_numeric(g["targets_successful"], errors="coerce").sum())
    ts, ts_lo, ts_hi = wilson(successful, attempted)

    mc = g["mission_completed_num"].dropna()
    mcp, mcl, mch = wilson(int(mc.sum()), len(mc))

    rh = g["returned_home_num"].dropna()
    rhp, rhl, rhh = wilson(int(rh.sum()), len(rh))

    ap = mean_sd_ci(g["mean_actual_path_length_m"])
    nt = mean_sd_ci(g["mean_navigation_time_s"])
    rp = mean_sd_ci(g["total_replans"])
    rc = mean_sd_ci(g["total_recoveries"])

    overall_rows.append({
        "method": m,
        "method_label": LABELS[m],
        "n_runs": len(g),
        "targets_successful": successful,
        "targets_attempted": attempted,
        "target_success_pct": ts,
        "target_success_ci95_low": ts_lo,
        "target_success_ci95_high": ts_hi,
        "mission_completion_pct": mcp,
        "mission_completion_ci95_low": mcl,
        "mission_completion_ci95_high": mch,
        "return_home_pct": rhp,
        "return_home_ci95_low": rhl,
        "return_home_ci95_high": rhh,
        "actual_path_m_mean": ap[0],
        "actual_path_m_sd": ap[1],
        "navigation_time_s_mean": nt[0],
        "navigation_time_s_sd": nt[1],
        "replans_per_run_mean": rp[0],
        "replans_per_run_sd": rp[1],
        "recoveries_per_run_mean": rc[0],
        "recoveries_per_run_sd": rc[1],
    })

overall = pd.DataFrame(overall_rows)
overall.to_csv(TABLES / "table06_overall_results.csv", index=False)

def save(fig, filename):
    fig.tight_layout()
    fig.savefig(FIGS / f"{filename}.png", dpi=300, bbox_inches="tight")
    fig.savefig(FIGS / f"{filename}.pdf", bbox_inches="tight")
    plt.close(fig)

def grouped_rate(metric, low, high, ylabel, title, filename):
    fig, ax = plt.subplots(figsize=(9.5, 5.4))
    x = np.arange(len(SCENARIOS))
    width = 0.19

    for i, m in enumerate(METHODS):
        d = scenario[scenario["method"] == m].set_index("scenario").reindex(SCENARIOS)
        y = d[metric].to_numpy(float)
        lo = d[low].to_numpy(float)
        hi = d[high].to_numpy(float)
        err = np.vstack([np.maximum(0, y-lo), np.maximum(0, hi-y)])
        ax.bar(x + (i-1.5)*width, y, width,
               label=LABELS[m], yerr=err, capsize=3)

    ax.set_xticks(x)
    ax.set_xticklabels(SCENARIOS)
    ax.set_ylim(0, 100)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid(axis="y", alpha=0.25)
    ax.legend(fontsize=8)
    save(fig, filename)

def grouped_cont(prefix, ylabel, title, filename):
    fig, ax = plt.subplots(figsize=(9.5, 5.4))
    x = np.arange(len(SCENARIOS))
    width = 0.19

    for i, m in enumerate(METHODS):
        d = scenario[scenario["method"] == m].set_index("scenario").reindex(SCENARIOS)
        y = d[f"{prefix}_mean"].to_numpy(float)
        lo = d[f"{prefix}_ci95_low"].to_numpy(float)
        hi = d[f"{prefix}_ci95_high"].to_numpy(float)

        lower = np.where(np.isnan(y) | np.isnan(lo), 0, np.maximum(0, y-lo))
        upper = np.where(np.isnan(y) | np.isnan(hi), 0, np.maximum(0, hi-y))
        err = np.vstack([lower, upper])

        ax.bar(x + (i-1.5)*width, y, width,
               label=LABELS[m], yerr=err, capsize=3)

    ax.set_xticks(x)
    ax.set_xticklabels(SCENARIOS)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid(axis="y", alpha=0.25)
    ax.legend(fontsize=8)
    save(fig, filename)

grouped_rate(
    "target_success_pct",
    "target_success_ci95_low",
    "target_success_ci95_high",
    "Target success (%)",
    "Target-success rate by method and scenario (95% CI)",
    "fig01_target_success"
)

grouped_rate(
    "mission_completion_pct",
    "mission_completion_ci95_low",
    "mission_completion_ci95_high",
    "Mission completion (%)",
    "Mission-completion rate by method and scenario (95% CI)",
    "fig02_mission_completion"
)

grouped_cont(
    "actual_path_m",
    "Actual path length (m)",
    "Mean actual path length by method and scenario (95% CI)",
    "fig03_actual_path"
)

grouped_cont(
    "navigation_time_s",
    "Navigation time (s)",
    "Mean navigation time by method and scenario (95% CI)",
    "fig04_navigation_time"
)

grouped_cont(
    "replans_per_run",
    "Replans per run",
    "Replanning activity by method and scenario (95% CI)",
    "fig05_replans"
)

grouped_cont(
    "recoveries_per_run",
    "Recovery events per run",
    "Recovery activity by method and scenario (95% CI)",
    "fig06_recoveries"
)

grouped_rate(
    "return_home_pct",
    "return_home_ci95_low",
    "return_home_ci95_high",
    "Return-home success (%)",
    "Return-home success by method and scenario (95% CI)",
    "fig07_return_home"
)

grouped_cont(
    "candidate_checks_per_run",
    "Candidate checks per run",
    "Candidate-search effort by method and scenario (95% CI)",
    "fig08_candidate_checks"
)

# Trade-off: overall target success vs actual traveled path.
fig, ax = plt.subplots(figsize=(7.4, 5.4))
for m in METHODS:
    r = overall[overall["method"] == m].iloc[0]
    ax.scatter(r["actual_path_m_mean"], r["target_success_pct"], s=70)
    ax.annotate(m, (r["actual_path_m_mean"], r["target_success_pct"]),
                xytext=(5,5), textcoords="offset points")
ax.set_xlabel("Mean actual path length (m)")
ax.set_ylabel("Target success (%)")
ax.set_ylim(0, 100)
ax.set_title("Robustness-efficiency trade-off")
ax.grid(alpha=0.25)
save(fig, "fig09_success_vs_actual_path")

# Paper 1 vs Paper 2 scenario target-success comparison.
fig, ax = plt.subplots(figsize=(10.5, 5.5))
comparison = comparison.sort_values(["scenario", "method"]).reset_index(drop=True)
labels = [f"{r.scenario}-{r.method}" for r in comparison.itertuples()]
x = np.arange(len(comparison))
width = 0.38
ax.bar(x-width/2, comparison["paper1_target_success_pct"], width,
       label="Paper 1 pilot (n=3/cell)")
ax.bar(x+width/2, comparison["paper2_target_success_pct"], width,
       label="Paper 2 (n=10/cell)")
ax.set_xticks(x)
ax.set_xticklabels(labels, rotation=45, ha="right")
ax.set_ylim(0, 100)
ax.set_ylabel("Target success (%)")
ax.set_title("Paper 1 pilot vs Paper 2: target-success rate")
ax.grid(axis="y", alpha=0.25)
ax.legend()
save(fig, "fig10_paper1_vs_paper2_target_success")

# Paper 1 vs Paper 2 mission completion.
fig, ax = plt.subplots(figsize=(10.5, 5.5))
ax.bar(x-width/2, comparison["paper1_mission_completion_pct"], width,
       label="Paper 1 pilot (n=3/cell)")
ax.bar(x+width/2, comparison["paper2_mission_completion_pct"], width,
       label="Paper 2 (n=10/cell)")
ax.set_xticks(x)
ax.set_xticklabels(labels, rotation=45, ha="right")
ax.set_ylim(0, 100)
ax.set_ylabel("Mission completion (%)")
ax.set_title("Paper 1 pilot vs Paper 2: mission-completion rate")
ax.grid(axis="y", alpha=0.25)
ax.legend()
save(fig, "fig11_paper1_vs_paper2_mission_completion")

# Overall target success.
fig, ax = plt.subplots(figsize=(8.4, 5.2))
x2 = np.arange(len(METHODS))
d = overall.set_index("method").reindex(METHODS)
y = d["target_success_pct"].to_numpy(float)
lo = d["target_success_ci95_low"].to_numpy(float)
hi = d["target_success_ci95_high"].to_numpy(float)
err = np.vstack([np.maximum(0,y-lo), np.maximum(0,hi-y)])
ax.bar(x2, y, yerr=err, capsize=4)
ax.set_xticks(x2)
ax.set_xticklabels([LABELS[m] for m in METHODS], rotation=18, ha="right")
ax.set_ylim(0,100)
ax.set_ylabel("Target success (%)")
ax.set_title("Overall Paper 2 target-success rate (95% CI)")
ax.grid(axis="y", alpha=0.25)
save(fig, "fig12_overall_target_success")

# Human-readable summary.
summary = OUT / "publication_summary.txt"
with summary.open("w") as f:
    f.write("PAPER 2 PUBLICATION RESULTS\n")
    f.write("="*78 + "\n\n")
    f.write("OVERALL RESULTS\n")
    f.write(overall.round(3).to_string(index=False))
    f.write("\n\nPAPER 1 VS PAPER 2 PRIMARY COMPARISON\n")
    f.write(comparison.round(2).to_string(index=False))
    f.write("\n")

print("=" * 78)
print("PUBLICATION PACKAGE CREATED")
print("=" * 78)
print("Tables :", TABLES)
print("Figures:", FIGS)
print("Summary:", summary)
print()
print("Files:")
for p in sorted(TABLES.iterdir()):
    print(" TABLE ", p.name)
for p in sorted(FIGS.glob("*.png")):
    print(" FIG   ", p.name)
