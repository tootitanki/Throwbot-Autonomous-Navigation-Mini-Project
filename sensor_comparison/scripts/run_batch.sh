#!/bin/bash
# Usage: ./run_batch.sh [results_dir] [n_seeds]   Runs the full grid: 4 sensor conditions (depth160 = depth camera at 160x120) x 3 obstacle speeds x n_seeds (paired seeds).
OUT=${1:-$HOME/throwbot_sensor_results}; N=${2:-6}
HERE="$(cd "$(dirname "$0")" && pwd)"
for seed in $(seq 1 $N); do for speed in 0.3 0.5 0.8; do for sensor in lidar lidar_fwd87 depth160 ultrasonic; do
  grep -q "\"run_id\": \"${sensor}_v${speed}_s${seed}\"" "$OUT/summary.jsonl" 2>/dev/null && continue   # resume support
  "$HERE/run_trial.sh" $sensor $speed $seed "$OUT" > /dev/null
done; done; done
echo done > "$OUT/BATCH_DONE"
