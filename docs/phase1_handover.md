# Phase 1 Handover Note — Robot and Sensor Validity

- **Status:** Complete (all 5 automated exit checks passed).
- **Packages Created:** `amr_description`, `amr_simulation`, `amr_bringup`.
- **Robot Geometry & Frames:** Differential-drive AMR (`0.45 m x 0.38 m` chassis, `0.08 m` wheel radius, `0.44 m` track width, `2 mm` caster clearance). Valid unique TF chain `odom -> base_footprint -> base_link -> base_scan`.
- **Sensors & Safeguards:** 360-ray 2D LiDAR on `/scan` (`base_scan` at `x = 0.15 m`), wheel-encoder odometry on `/odom`, evaluation-only ground truth on `/ground_truth/odom`, and `cmd_vel_watchdog` enforcing a `0.20 m/s` linear cap and `0.25 s` stale-command timeout.
- **Verification Command:** `python3 scripts/verify_phase1.py`
