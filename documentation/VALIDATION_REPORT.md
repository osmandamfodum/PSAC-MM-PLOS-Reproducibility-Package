# Final dataset validation

- Final manifest rows: **120**
- Unique run IDs: **120**
- Classification: **120 `VALID_QUANTITATIVE_RUN`**
- Runs per scenario: S1=40, S2=40, S3=40
- Runs per method: B1=30, B2=30, B3=30, P=30
- Runs in every scenario-method cell: **10**
- Duplicate run IDs: **0**
- Duplicate scenario-method-seed combinations: **0**
- Missing accepted event logs: **0**
- Event-count mismatches: **0**
- Consolidated event records: **52,271**
- Consolidated JSON parse failures: **0**
- Derived run rows: **120**
- Derived target rows: **360**
- Target rows per run: **3 for every run**
- Candidate rows: **6,586 across the 90 candidate-based runs**

Outcome failures were retained: 92 missions did not complete, 70 runs did not return home, and 30 valid runs serviced zero targets. These are valid scientific observations, not infrastructure exclusions.

The manifest and consolidated event log agree on all 120 run IDs. The original consolidation utility that created these two final files from per-attempt logs was not found as a separate retained script; this limitation is documented rather than reconstructed.
