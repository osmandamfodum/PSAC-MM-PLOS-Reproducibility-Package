#!/usr/bin/env python3
from pathlib import Path
import shutil
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

ROOT = Path(
    "/home/drosmanalhussein/robot_ws2/benchmark_results/"
    "navigation_v2/release_candidate_rc12/paper2_analysis"
)
SRC_PUB = ROOT / "publication"
FINAL = ROOT / "publication_final"
SRC = ROOT / "paper1_style"

METHODS = ["B1", "B2", "B3", "P"]
SCENARIOS = ["S1", "S2", "S3"]
LABELS = {
    "B1": "Direct-to-target",
    "B2": "Single-candidate",
    "B3": "No persistent management",
    "P": "Proposed (PSAC-MM)",
}

if FINAL.exists():
    shutil.rmtree(FINAL)
shutil.copytree(SRC_PUB, FINAL)

TABLES = FINAL / "tables"
FIGS = FINAL / "figures"

rm = pd.read_csv(SRC / "run_metrics.csv")
rm["run_success_pct"] = (
    100.0
    * pd.to_numeric(rm["targets_successful"], errors="coerce")
    / pd.to_numeric(rm["targets_attempted"], errors="coerce")
)

def stable_seed(scenario, method):
    smap = {"S1": 1, "S2": 2, "S3": 3, "ALL": 9}
    mmap = {"B1": 1, "B2": 2, "B3": 3, "P": 4}
    return 20260921 + 100 * smap[scenario] + mmap[method]

def cluster_bootstrap_ci(values, seed, B=20000):
    x = pd.to_numeric(pd.Series(values), errors="coerce").dropna().to_numpy(float)
    if len(x) == 0:
        return np.nan, np.nan, np.nan
    estimate = float(np.mean(x))
    if len(x) == 1:
        return estimate, estimate, estimate
    rng = np.random.default_rng(seed)
    idx = rng.integers(0, len(x), size=(B, len(x)))
    means = x[idx].mean(axis=1)
    lo, hi = np.percentile(means, [2.5, 97.5])
    return estimate, float(lo), float(hi)

dist = rm[[
    "run_id", "scenario", "method", "seed",
    "targets_successful", "targets_attempted", "run_success_pct"
]].sort_values(["scenario", "method", "seed"])
dist.to_csv(TABLES / "table07_run_level_target_success.csv", index=False)

t1 = pd.read_csv(TABLES / "table01_scenario_method_full.csv")
t2 = pd.read_csv(TABLES / "table02_primary_results.csv")

for s in SCENARIOS:
    for m in METHODS:
        vals = rm.loc[
            (rm["scenario"] == s) & (rm["method"] == m),
            "run_success_pct"
        ]
        est, lo, hi = cluster_bootstrap_ci(vals, stable_seed(s, m))

        for df in (t1, t2):
            mask = (df["scenario"] == s) & (df["method"] == m)
            df.loc[mask, "target_success_pct"] = est
            df.loc[mask, "target_success_ci95_low"] = lo
            df.loc[mask, "target_success_ci95_high"] = hi

t1.to_csv(TABLES / "table01_scenario_method_full.csv", index=False)
t2.to_csv(TABLES / "table02_primary_results.csv", index=False)

t6 = pd.read_csv(TABLES / "table06_overall_results.csv")
for m in METHODS:
    vals = rm.loc[rm["method"] == m, "run_success_pct"]
    est, lo, hi = cluster_bootstrap_ci(vals, stable_seed("ALL", m))
    mask = t6["method"] == m
    t6.loc[mask, "target_success_pct"] = est
    t6.loc[mask, "target_success_ci95_low"] = lo
    t6.loc[mask, "target_success_ci95_high"] = hi

t6.to_csv(TABLES / "table06_overall_results.csv", index=False)

def save(fig, name):
    fig.tight_layout()
    fig.savefig(FIGS / f"{name}.png", dpi=300, bbox_inches="tight")
    fig.savefig(FIGS / f"{name}.pdf", bbox_inches="tight")
    plt.close(fig)

fig, ax = plt.subplots(figsize=(9.5, 5.4))
x = np.arange(len(SCENARIOS))
width = 0.19

for i, m in enumerate(METHODS):
    d = (
        t2[t2["method"] == m]
        .set_index("scenario")
        .reindex(SCENARIOS)
    )
    y = d["target_success_pct"].to_numpy(float)
    lo = d["target_success_ci95_low"].to_numpy(float)
    hi = d["target_success_ci95_high"].to_numpy(float)
    err = np.vstack([
        np.maximum(0, y - lo),
        np.maximum(0, hi - y)
    ])
    ax.bar(
        x + (i - 1.5) * width,
        y,
        width,
        label=LABELS[m],
        yerr=err,
        capsize=3
    )

ax.set_xticks(x)
ax.set_xticklabels(SCENARIOS)
ax.set_ylim(0, 105)
ax.set_ylabel("Target success (%)")
ax.set_title("Target-success rate by method and scenario")
ax.grid(axis="y", alpha=0.25)
ax.legend(fontsize=8)
ax.text(
    0.01, -0.15,
    "Error bars: 95% run-cluster bootstrap CI (10 independent runs per scenario-method cell).",
    transform=ax.transAxes,
    fontsize=8
)
save(fig, "fig01_target_success")

fig, ax = plt.subplots(figsize=(8.4, 5.2))
d = t6.set_index("method").reindex(METHODS)
x2 = np.arange(len(METHODS))
y = d["target_success_pct"].to_numpy(float)
lo = d["target_success_ci95_low"].to_numpy(float)
hi = d["target_success_ci95_high"].to_numpy(float)
err = np.vstack([
    np.maximum(0, y - lo),
    np.maximum(0, hi - y)
])

ax.bar(x2, y, yerr=err, capsize=4)
ax.set_xticks(x2)
ax.set_xticklabels(
    [LABELS[m] for m in METHODS],
    rotation=18,
    ha="right"
)
ax.set_ylim(0, 105)
ax.set_ylabel("Target success (%)")
ax.set_title("Overall Paper 2 target-success rate")
ax.grid(axis="y", alpha=0.25)
ax.text(
    0.01, -0.22,
    "Error bars: 95% run-cluster bootstrap CI (30 independent runs per method).",
    transform=ax.transAxes,
    fontsize=8
)
save(fig, "fig12_overall_target_success")

note = FINAL / "STATISTICAL_METHODS.txt"
note.write_text(
    """PAPER 2 STATISTICAL REPORTING METHODS

Independent experimental unit
-----------------------------
One benchmark run is treated as the independent experimental unit.
Each scenario-method cell contains 10 independent runs, each with 3 targets.

Target-success rate
-------------------
Point estimate:
    total successful targets / total attempted targets
Because every final run contains exactly 3 attempted targets, this is equivalent
to the mean per-run target-success percentage.

Uncertainty:
    95% percentile bootstrap confidence interval obtained by resampling complete
    runs with replacement (20,000 bootstrap samples). Targets are not resampled
    independently.

Important boundary case:
    If all 10 observed runs have the same success percentage (for example all
    100% or all 0%), the empirical run-cluster bootstrap interval is degenerate.
    This means no between-run variation was observed in this campaign; it must
    not be interpreted as proof of zero population uncertainty.

Mission completion and return-home
----------------------------------
Wilson 95% confidence intervals on binary run-level outcomes.

Continuous run-level metrics
----------------------------
Mean, sample SD, and 95% t-based confidence intervals across runs.

Cross-paper comparison
----------------------
Paper 1 and Paper 2 are separate experimental campaigns. Differences are
reported descriptively and are not interpreted as paired causal effects.
"""
)

print("=" * 78)
print("FINAL PUBLICATION PACKAGE CREATED")
print("=" * 78)
print("Directory:", FINAL)
print()
print("Updated target-success CI method: run-cluster bootstrap (20,000 resamples)")
print()
print("Scenario-level target success:")
print(
    t2[[
        "scenario", "method", "target_success_pct",
        "target_success_ci95_low", "target_success_ci95_high"
    ]].round(2).to_string(index=False)
)
print()
print("Overall target success:")
print(
    t6[[
        "method", "target_success_pct",
        "target_success_ci95_low", "target_success_ci95_high"
    ]].round(2).to_string(index=False)
)
print()
print("Statistical note:", note)
