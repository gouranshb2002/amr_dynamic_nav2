# Phase 3 Handover Note — Dynamic Warehouse & Baseline Benchmark

- **Status:** Complete (Baseline: 3/12 = 25.0%, 8 collisions, 1 timeout in `results/phase3_baseline_dynamic/`).
- **Findings:** Unmodified Nav2 lacks velocity projection (`BaseObstacle.scale=0.02`), cannot yield/step back from sweeping agents, and exhibits `0.08–0.16 m` AMCL terminal offset.
