# Reproduction steps

## Rebuild derived event tables

From the package root, using Python with pandas, NumPy and Matplotlib installed:

```bash
python3 code/analysis/benchmark_analysis.py \
  --events data/raw_or_event_level/paper2_final_events.jsonl \
  --out reproduced_analysis
```

This command is directly supported by the retained analysis script. Compare the resulting CSV files with `data/run_level/`.

## Publication tables and figures

`paper2_publication_figures.py` and `paper2_finalize_cluster_ci.py` are the exact final scripts. Their original `ROOT` constants refer to the author's campaign directory. For a portable rerun, edit only `ROOT` in a working copy to point to the package's reproduced analysis directory. Run them in this order:

```bash
python3 code/analysis/paper2_publication_figures.py
python3 code/analysis/paper2_finalize_cluster_ci.py
```

Do not alter statistical logic. The final target-success CI uses 20,000 complete-run bootstrap resamples.

## Full ROS 2 campaign

The retained `paper2_batch_runner_v5.py` is the final campaign runner. It contains absolute paths and host-readiness requirements from the original machine, so it must be reviewed and adapted to a new ROS 2 Humble workspace before execution. Its supported command-line interface can be inspected with:

```bash
python3 code/simulation/paper2_batch_runner_v5.py --help
```

The final campaign used headless Gazebo, no RViz, non-composed Nav2 with ordered manual lifecycle startup, the supplied original farm map, and seeds 1001–1010.
