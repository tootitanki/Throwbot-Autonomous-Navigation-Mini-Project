# Sensor comparison: detection and avoidance of a moving obstacle (simulation)

Question: how do a 2D lidar, a narrow-cone ultrasonic-style range sensor and a depth camera differ in detecting and avoiding a crossing 0.5 m box, with everything else held fixed (Nav2 stack, planner settings, world, goal, seeds)?
The metric is detection and avoidance performance, not sensor latency.

This is a simulation study of modelled sensors with configured properties. It is not evidence about real hardware.

## What was run
- Stack: ROS 2 Humble, Gazebo (Ignition) Fortress 6.x, Nav2, headless, software rendering. ROS 2 Jazzy was not tested.
- World: 12 x 8 m room, fixed boxes, one 0.5 m box on a rail crossing the route. Robot 0.40 x 0.30 m, start world (-4.5, 0), goal (4.5, 0).
- Conditions (n = 18 each: 3 obstacle speeds 0.3/0.5/0.8 m/s x seeds 1-6, 72 trials total, paired seeds):
  - `lidar`: 2D lidar, 10 Hz.
  - `lidar_fwd87`: the same lidar restricted to a forward sector.
  - `depth160`: depth camera at **160x120**, 15 Hz configured, FOV and range unchanged. This is a changed-resolution condition (see the depth transport note), not the same as a 320x240 camera.
  - `ultrasonic`: an **approximation**. 9 rays in a 15 degree cone feed Nav2's range layer. Gazebo Fortress has no sonar sensor.
- Obstacle visibility depends on the costmap marking range and the shared costmap update rate (about 2.5 Hz in all conditions), not only on the sensor.

## Key assumptions
1. **Ground-truth localization is an experimental assumption.** The robot pose comes from Gazebo's model pose (OdometryPublisher), with a static map->odom offset. This isolates the sensor effect. It says nothing about real-world odometry or SLAM performance.
2. Wheel odometry is not used. The first batch (v1) used wheel odometry; it diverged from truth by up to 1.86 m, so its depth, ultrasonic and contact numbers are invalid. v1 is kept only as a labelled diagnostic in `diagnostics/v1_odometry_INVALID/`.
3. The body-versus-box gap is computed in the logger from the true oriented 0.40 x 0.30 m body against the 0.5 x 0.5 m box.

## Results (v2, 72 trials, 0 excluded)
All 72 goals reported SUCCEEDED. No aborts, no timeouts. Goal success is not the same as clean avoidance.

| condition | n | logged gap <= 1 cm | logged gap <= 5 cm | median detect lag (sim s) | mean tracking recall | mean phantom cells |
|---|---|---|---|---|---|---|
| lidar | 18 | 11 | 12 | 0.23 | 0.61 | 13 |
| lidar_fwd87 | 18 | 11 | 11 | 0.22 | 0.82 | 30 |
| depth160 | 18 | 18 | 18 | 0.12 | 0.64 | 51 |
| ultrasonic | 18 | 13 | 14 | 0.28 | 0.64 | 0.3 |

Per-speed tables: `analysis/table_by_sensor_speed.csv`; all trials: `analysis/summary_all.csv`; raw rows: `results_v2/summary.jsonl`. Wall-clock rates are in the tables too; sim-clock and wall-clock metrics are kept separate.

What this does and does not say:
- No collision percentages, no sensor ranking and no claim that one sensor type is better. Samples are small (n = 18 per condition) and the conditions differ in more than the sensor type (depth160 resolution, shared costmap rate, FOV, the ultrasonic approximation).
- "Logged gap <= 1 cm" is a logged geometric proximity, **not confirmed physical contact**. In the instrumented replays a minimum body-box gap of 0 with penetration 0 does not by itself prove contact. It is described as zero geometric separation in the instrumented replay. There is no independent Gazebo contact/collision evidence.
- Explanations for the differences (shared 2.5 Hz costmap limit, stale marks in depth160 behind its high phantom count, its FOV) are untested hypotheses.

## Validation and limits
- Heartbeat log: no gaps > 5 s, so no trial was excluded. Exclusion rule was fixed in advance: only independent pause evidence, applied equally across sensors and seeds.
- Infra faults (`results_v2/infra_retries.log`): ultrasonic 0.3 m/s seed 6 had two attempts that produced no result (22:05:14, 22:08:12) before a clean third attempt. The earlier attempts are not counted.
- Truth replays (`diagnostics/truth_replays/`, `scripts/gt_check.py`): instrumented replays of depth160 0.5 m/s seed 2 and lidar 0.5 m/s seed 1. Max logged-pose vs truth difference was 0.053 m (depth160 replay) and 0.033 m (lidar replay). The lidar replay gap (0.131 m truth, 0.133 m logger) differs from the batch row (0.10 m) because it is a replay, not the same trial. Only the raw output of the depth160 replay (`gt_v2_depth160_s2_replay.json`, from `scripts/gt_check.py`) is kept in this repo; the lidar replay numbers (0.131 m truth gap, 0.133 m logger, 0.033 m max diff) are console observations whose raw file was not preserved, so treat them as unreproduced notes, not preserved evidence. Limits: 0.1 s binning, about 5 cm timing mismatch. One probe was aborted and two probes accidentally ran concurrently; both are diagnostics only and the aborted probe's cause is not established.
- Depth transport finding: a 320x240 depth stream at a configured 15 Hz reached about 13.6 Hz for a RELIABLE reader but 0.1-0.2 Hz for a BEST_EFFORT reader (the QoS Nav2 and the logger use). The cause is not proven. A relay raised this to about 5.7 Hz. 160x120 delivered the configured rate, so it was used as a bounded fix. The 320x240 behaviour is a diagnostic only.
- Single runs, one world, no hardware.

## Omitted from this GitHub repo
This repo is a trimmed copy, not the full raw archive. Omitted: the four screen-recording mp4s (about 8 MB) and the 72 per-trial trace CSVs (`results_v2/traces/`). They are in a private Google Drive folder (Throwbot Autonomous Navigation Mini Project / Sensor Comparison, owner's Drive) that is not public. `recordings/contact_sheet.png` and the per-condition `trial_result_*.jsonl` files are kept. Everything else described here is included.

Two empty marker files (`src/throwbot_sensors/resource/throwbot_sensors` and `src/throwbot_sensors/throwbot_sensors/__init__.py`) are not in this GitHub copy because the web upload cannot carry empty files. Create them (empty) before `colcon build`: `touch` both paths.

## Recordings (`recordings/`, videos not in this repo)
Real Gazebo GUI + RViz screen captures (Xvfb, ffmpeg), one per condition, 0.5 m/s seed 2, about 38 s each. `contact_sheet.png` has a mid-run frame of each. They were recorded with GUI software rendering, so rates and RTF differ from the headless batch. They illustrate single runs and are not evidence for a comparison. Each trial's logged result is in `trial_result_<condition>.jsonl`. All four logged the goal as SUCCEEDED with logged gap 0.

## How to read the tables and figures
- `succeeded_gap_gt_1cm` and `succeeded_gap_le_1cm_logged` split the 72 SUCCEEDED goals by the logged minimum body-box gap. They do not mean "reached" versus "failed": every goal succeeded.
- Tables give medians or means as named in each column (detect lag and gap: median; tracking recall and phantom cells: mean). Per-speed rows have n = 6.
- In the figures, lines are the median over 6 seeds at each speed and the dots are the individual trials. Small samples; the lines do not show that one sensor is better. The tracking-recall figure uses the median per speed, the table uses the mean over all 18 trials, so they are not the same statistic.

## Reproducing (needs setup, not copy-and-run)
Tested only in the environment used for this study: ROS 2 Humble in a micromamba environment, Gazebo Fortress, Nav2, headless with software rendering, 2 CPU cores. Needs `ros-humble-ros-gz`, `navigation2`, `nav2-bringup`, `xacro`; for recordings also Xvfb, xdotool, ffmpeg and RViz.
1. Build the package: put `src/throwbot_sensors` in a colcon workspace and run `colcon build`.
2. `scripts/run_trial.sh` and `scripts/run_batch.sh` locate `install/setup.bash` relative to the script folder, so copy them into the workspace root (next to `install/`). They default to `$HOME/throwbot_sensor_results` for output. Run `./run_batch.sh <results_dir> 6`.
3. `scripts/analyse.py <results_dir> <out_dir>` needs pandas, numpy, matplotlib, tabulate.
4. `scripts/rec_sensor.sh` and `scripts/diag_probe.sh` have hardcoded paths from the study machine (`/tmp/env.sh`, `/tmp/study/ws`, `/tmp/study/orig.rviz` which is Nav2's stock RViz config). Edit those before use. `gt_check.py` and `recheck_overlap.py` also expect the study's file layout.
Not tested: ROS 2 Jazzy, other machines, a real GPU.
