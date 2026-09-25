# Persistent Safety-Aware Candidate Approach and Mission Management (PSAC-MM)

This is the reproducibility package for the expanded UAV–UGV precision-agriculture benchmark reported in the associated PLOS ONE manuscript.

## Experimental design

- **S1:** open/static navigation.
- **S2:** constrained crop-row navigation.
- **S3:** constrained navigation with dynamic obstacles.
- **B1:** direct-to-target navigation.
- **B2:** fixed-offset single candidate.
- **B3:** bounded multi-candidate planning and safety filtering without persistent target management.
- **P:** complete PSAC-MM with persistent target lifecycle, multi-candidate evaluation, safety filtering, bounded replanning and bounded recovery.

The final dataset contains 4 methods × 3 scenarios × 10 runs = **120 missions**. Each mission has 3 target opportunities, giving **360 target observations**.

## Directory contents

- `data/raw_or_event_level/`: consolidated structured event log and the manifest selecting the 120 accepted attempts.
- `data/run_level/`: derived run-, target-, candidate- and summary-level CSV files.
- `data/tables/`: final publication and manuscript tables.
- `data/figure_source_data/`: CSV inputs used by the publication figure scripts.
- `code/analysis/`: event extraction, publication-table/figure generation and final cluster-bootstrap CI code.
- `code/mission_management/`: the frozen RC12 PSAC-MM mission-management implementation.
- `code/simulation/`: final batch runner, UAV scanner and dynamic-obstacle controller.
- `code/configuration/`: Nav2, launch, map, world, URDF and ROS package configuration used to interpret the experiment.
- `documentation/`: design, methods, data dictionary, validation and reproduction guidance.

## Result provenance

- Target success, approach error, path length, navigation time, replans, recoveries and attempts are derived from `paper2_final_events.jsonl` by `benchmark_analysis.py`.
- Mission completion, return-home and collision fields are available in `runs_derived.csv` and `run_metrics.csv`.
- Candidate checks are derived from `candidate_events_derived.csv`.
- Publication confidence intervals and table values are in `data/tables/publication_final/`.
- `paper2_finalize_cluster_ci.py` implements the final run-cluster bootstrap confidence intervals for target success.

## Important execution note

The final campaign runner explicitly selected `my_farm_map.yaml` and `my_farm_map.pgm`. The generic launch files contain a different default map name, so reproductions should follow the final runner rather than relying on launch defaults.

See `documentation/REPRODUCTION_STEPS.md` for commands supported by the retained scripts.
