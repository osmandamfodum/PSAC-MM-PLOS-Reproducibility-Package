# Data dictionary

The descriptions below use field names present in the retained files. Blank or NaN values mean the quantity was unavailable or not applicable; they are not zero.

## `data/raw_or_event_level/paper2_valid_run_manifest.csv`

- `run_id`: Unique final mission identifier.
- `scenario`: Scenario code S1, S2 or S3.
- `method`: Method code B1, B2, B3 or P.
- `seed`: Configured campaign seed.
- `attempt`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `attempt_dir`: Original absolute directory of the accepted attempt.
- `event_count`: Number of event records in the accepted attempt.
- `classification`: Attempt validity classification.
- `nav2_composition`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `nav2_startup_mode`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `runner_cleanup_mode`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `mission_manager_sha256`: Retained analysis field; see the generating script and neighboring fields for its computation.

## `data/run_level/candidate_events_derived.csv`

- `run_id`: Unique final mission identifier.
- `scenario`: Scenario code S1, S2 or S3.
- `method`: Method code B1, B2, B3 or P.
- `target_id`: Target identifier from the mission event stream.
- `attempt`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `candidate_id`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `angle_deg`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `radius_m`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `x`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `y`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `nav_precheck_safe`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `uav_safe`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `planner_feasible`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `path_length_m`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `planning_time_s`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `rejected_reason`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `selected`: Retained analysis field; see the generating script and neighboring fields for its computation.

## `data/run_level/candidate_summary.csv`

- `scenario`: Scenario code S1, S2 or S3.
- `method`: Method code B1, B2, B3 or P.
- `candidates_evaluated`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `nav_precheck_safe`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `uav_safe`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `planner_feasible`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `selected`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `mean_candidate_path_m`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `candidate_feasibility_pct`: Retained analysis field; see the generating script and neighboring fields for its computation.

## `data/run_level/event_counts.csv`

- `event`: Structured event type.
- `count`: Retained analysis field; see the generating script and neighboring fields for its computation.

## `data/run_level/method_scenario_summary.csv`

- `scenario`: Scenario code S1, S2 or S3.
- `method`: Method code B1, B2, B3 or P.
- `method_label`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `n_runs`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `mission_completion_pct`: Percentage of missions completed.
- `target_success_pct`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `path_length_m_mean`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `path_length_m_sd`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `path_length_m_n`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `actual_path_length_m_mean`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `actual_path_length_m_sd`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `actual_path_length_m_n`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `navigation_time_s_mean`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `navigation_time_s_sd`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `navigation_time_s_n`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `approach_error_m_mean`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `approach_error_m_sd`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `approach_error_m_n`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `planning_time_s_mean`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `planning_time_s_sd`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `planning_time_s_n`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `replans_per_run_mean`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `replans_per_run_sd`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `replans_per_run_n`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `recoveries_per_run_mean`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `recoveries_per_run_sd`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `recoveries_per_run_n`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `attempts_per_target_mean`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `attempts_per_target_sd`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `attempts_per_target_n`: Retained analysis field; see the generating script and neighboring fields for its computation.

## `data/run_level/overall_summary.csv`

- `method`: Method code B1, B2, B3 or P.
- `method_label`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `n_runs`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `mission_completion_pct`: Percentage of missions completed.
- `target_success_pct`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `path_length_mean_m`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `path_length_sd_m`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `navigation_time_mean_s`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `navigation_time_sd_s`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `approach_error_mean_m`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `approach_error_sd_m`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `replans_per_run_mean`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `replans_per_run_sd`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `recoveries_per_run_mean`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `recoveries_per_run_sd`: Retained analysis field; see the generating script and neighboring fields for its computation.

## `data/run_level/run_metrics.csv`

- `run_id`: Unique final mission identifier.
- `scenario`: Scenario code S1, S2 or S3.
- `method`: Method code B1, B2, B3 or P.
- `seed`: Configured campaign seed.
- `target_count`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `start_x`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `start_y`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `home_x`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `home_y`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `mission_start_time`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `mission_end_time`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `mission_completed`: Whether all three targets were successfully serviced.
- `returned_home`: Whether the return-home criterion was satisfied.
- `manual_intervention`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `collision_count`: Number of logged collision events.
- `software_commit`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `targets_attempted`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `targets_successful`: Number of successful targets in a run.
- `mean_path_length_m`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `mean_actual_path_length_m`: Mean target-level actual path length within a run.
- `mean_navigation_time_s`: Mean target navigation time within a run.
- `mean_total_target_time_s`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `mean_approach_error_m`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `mean_planning_time_s`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `total_replans`: Run-level sum of replans.
- `total_recoveries`: Run-level sum of recoveries.
- `mean_attempt_count`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `target_success_rate_pct`: Retained analysis field; see the generating script and neighboring fields for its computation.

## `data/run_level/runs_derived.csv`

- `run_id`: Unique final mission identifier.
- `scenario`: Scenario code S1, S2 or S3.
- `method`: Method code B1, B2, B3 or P.
- `seed`: Configured campaign seed.
- `target_count`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `start_x`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `start_y`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `home_x`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `home_y`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `mission_start_time`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `mission_end_time`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `mission_completed`: Whether all three targets were successfully serviced.
- `returned_home`: Whether the return-home criterion was satisfied.
- `manual_intervention`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `collision_count`: Number of logged collision events.
- `software_commit`: Retained analysis field; see the generating script and neighboring fields for its computation.

## `data/run_level/target_events_derived.csv`

- `run_id`: Unique final mission identifier.
- `target_id`: Target identifier from the mission event stream.
- `target_x`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `target_y`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `target_order`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `target_start_time`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `goal_accepted_time`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `target_reached_time`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `treatment_time`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `target_completed_time`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `target_success`: Whether the target was successfully serviced.
- `attempt_count`: Number of attempts used for a target.
- `replan_count`: Target-level number of replans.
- `recovery_count`: Target-level number of recoveries.
- `planned_path_length_m`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `actual_path_length_m`: Odometry-derived path length for a target navigation.
- `approach_error_m`: Final target-distance error for a target.
- `final_target_distance_m`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `final_service_distance_m`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `planning_time_s`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `navigation_time_s`: Target navigation duration in simulation seconds.
- `total_target_time_s`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `failure_reason`: Recorded target failure reason when unsuccessful.

## `data/tables/publication_final/table01_scenario_method_full.csv`

- `scenario`: Scenario code S1, S2 or S3.
- `method`: Method code B1, B2, B3 or P.
- `method_label`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `n_runs`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `targets_successful`: Number of successful targets in a run.
- `targets_attempted`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `target_success_pct`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `target_success_ci95_low`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `target_success_ci95_high`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `missions_completed`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `mission_completion_pct`: Percentage of missions completed.
- `mission_completion_ci95_low`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `mission_completion_ci95_high`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `returned_home_runs`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `return_home_pct`: Percentage of runs satisfying return-home.
- `return_home_ci95_low`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `return_home_ci95_high`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `collision_count_mean`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `actual_path_m_mean`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `actual_path_m_sd`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `actual_path_m_ci95_low`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `actual_path_m_ci95_high`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `actual_path_m_n`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `navigation_time_s_mean`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `navigation_time_s_sd`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `navigation_time_s_ci95_low`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `navigation_time_s_ci95_high`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `navigation_time_s_n`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `approach_error_m_mean`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `approach_error_m_sd`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `approach_error_m_ci95_low`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `approach_error_m_ci95_high`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `approach_error_m_n`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `planning_time_s_mean`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `planning_time_s_sd`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `planning_time_s_ci95_low`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `planning_time_s_ci95_high`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `planning_time_s_n`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `replans_per_run_mean`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `replans_per_run_sd`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `replans_per_run_ci95_low`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `replans_per_run_ci95_high`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `replans_per_run_n`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `recoveries_per_run_mean`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `recoveries_per_run_sd`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `recoveries_per_run_ci95_low`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `recoveries_per_run_ci95_high`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `recoveries_per_run_n`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `attempts_per_target_mean`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `attempts_per_target_sd`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `attempts_per_target_ci95_low`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `attempts_per_target_ci95_high`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `attempts_per_target_n`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `candidate_checks_per_run_mean`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `candidate_checks_per_run_sd`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `candidate_checks_per_run_ci95_low`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `candidate_checks_per_run_ci95_high`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `candidate_checks_per_run_n`: Retained analysis field; see the generating script and neighboring fields for its computation.

## `data/tables/publication_final/table02_primary_results.csv`

- `scenario`: Scenario code S1, S2 or S3.
- `method`: Method code B1, B2, B3 or P.
- `n_runs`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `targets_successful`: Number of successful targets in a run.
- `targets_attempted`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `target_success_pct`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `target_success_ci95_low`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `target_success_ci95_high`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `missions_completed`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `mission_completion_pct`: Percentage of missions completed.
- `mission_completion_ci95_low`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `mission_completion_ci95_high`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `returned_home_runs`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `return_home_pct`: Percentage of runs satisfying return-home.
- `return_home_ci95_low`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `return_home_ci95_high`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `collision_count_mean`: Retained analysis field; see the generating script and neighboring fields for its computation.

## `data/tables/publication_final/table03_navigation_efficiency.csv`

- `scenario`: Scenario code S1, S2 or S3.
- `method`: Method code B1, B2, B3 or P.
- `n_runs`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `actual_path_m_mean`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `actual_path_m_sd`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `actual_path_m_ci95_low`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `actual_path_m_ci95_high`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `navigation_time_s_mean`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `navigation_time_s_sd`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `navigation_time_s_ci95_low`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `navigation_time_s_ci95_high`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `approach_error_m_mean`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `approach_error_m_sd`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `approach_error_m_ci95_low`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `approach_error_m_ci95_high`: Retained analysis field; see the generating script and neighboring fields for its computation.

## `data/tables/publication_final/table04_planning_recovery.csv`

- `scenario`: Scenario code S1, S2 or S3.
- `method`: Method code B1, B2, B3 or P.
- `n_runs`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `planning_time_s_mean`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `planning_time_s_sd`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `replans_per_run_mean`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `replans_per_run_sd`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `recoveries_per_run_mean`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `recoveries_per_run_sd`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `attempts_per_target_mean`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `attempts_per_target_sd`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `candidate_checks_per_run_mean`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `candidate_checks_per_run_sd`: Retained analysis field; see the generating script and neighboring fields for its computation.

## `data/tables/publication_final/table05_paper1_vs_paper2_primary.csv`

- `scenario`: Scenario code S1, S2 or S3.
- `method`: Method code B1, B2, B3 or P.
- `paper1_target_success_pct`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `paper1_mission_completion_pct`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `n_runs`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `paper2_target_success_pct`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `paper2_mission_completion_pct`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `target_success_change_pp`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `mission_completion_change_pp`: Retained analysis field; see the generating script and neighboring fields for its computation.

## `data/tables/publication_final/table06_overall_results.csv`

- `method`: Method code B1, B2, B3 or P.
- `method_label`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `n_runs`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `targets_successful`: Number of successful targets in a run.
- `targets_attempted`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `target_success_pct`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `target_success_ci95_low`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `target_success_ci95_high`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `mission_completion_pct`: Percentage of missions completed.
- `mission_completion_ci95_low`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `mission_completion_ci95_high`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `return_home_pct`: Percentage of runs satisfying return-home.
- `return_home_ci95_low`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `return_home_ci95_high`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `actual_path_m_mean`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `actual_path_m_sd`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `navigation_time_s_mean`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `navigation_time_s_sd`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `replans_per_run_mean`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `replans_per_run_sd`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `recoveries_per_run_mean`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `recoveries_per_run_sd`: Retained analysis field; see the generating script and neighboring fields for its computation.

## `data/tables/publication_final/table07_run_level_target_success.csv`

- `run_id`: Unique final mission identifier.
- `scenario`: Scenario code S1, S2 or S3.
- `method`: Method code B1, B2, B3 or P.
- `seed`: Configured campaign seed.
- `targets_successful`: Number of successful targets in a run.
- `targets_attempted`: Retained analysis field; see the generating script and neighboring fields for its computation.
- `run_success_pct`: Retained analysis field; see the generating script and neighboring fields for its computation.
