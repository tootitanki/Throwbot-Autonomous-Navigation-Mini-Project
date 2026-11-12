#!/bin/bash
# Usage: ./run_trial.sh <sensor: lidar|lidar_fwd87|depth|ultrasonic> <obstacle speed m/s> <seed> [results_dir]
# Headless trial (software rendering under Xvfb). Appends one JSON line to <results_dir>/summary.jsonl.
# Infrastructure faults (map server not active, goal never accepted) are not results: the trial is repeated (max 2 retries)
# and the fault is noted in $OUT/infra_retries.log. Real failures (collision, abort, timeout) are always kept.
if [ -z "$TRIAL_RETRY" ]; then TRIAL_RETRY=0; fi
cleanup() {  # kill every process a trial can leave behind (stale nodes break the next trial)
  for p in "ign gazebo" ruby parameter_bridge static_transform_publisher lifecycle_manager map_server controller_server planner_server \
           smoother_server behavior_server bt_navigator waypoint_follower velocity_smoother robot_state_publisher range_from_scan \
           obstacle_mover study_logger rviz2 "ros2 launch" Xvfb; do pkill -9 -f "$p" 2>/dev/null; done
  sleep 2; rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
}
SENSOR=${1:-lidar}; SPEED=${2:-0.5}; SEED=${3:-1}; OUT=${4:-$HOME/throwbot_sensor_results}
RID="${SENSOR}_v${SPEED}_s${SEED}"
HERE="$(cd "$(dirname "$0")" && pwd)"
[ -n "$ROS_DISTRO" ] || source /opt/ros/humble/setup.bash
source "$HERE/install/setup.bash"
export LIBGL_ALWAYS_SOFTWARE=1
mkdir -p "$OUT/logs"
cleanup
setsid xvfb-run -a -s "-screen 0 1280x720x24" ros2 launch throwbot_sensors study.launch.py \
    sensor:=$SENSOR speed:=$SPEED seed:=$SEED gz_extra:="-s" out_dir:="$OUT" run_id:=$RID > "$OUT/logs/$RID.log" 2>&1 &
PG=$!
for i in $(seq 1 120); do
  grep -q "STUDY_RESULT" "$OUT/logs/$RID.log" && break
  kill -0 $PG 2>/dev/null || break
  sleep 3
done
grep "STUDY_RESULT" "$OUT/logs/$RID.log" | tail -1
kill -INT -- -$PG 2>/dev/null; sleep 4
cleanup
sleep 2
if grep -q "no map received" "$OUT/logs/$RID.log" || ! grep -q "STUDY_RESULT" "$OUT/logs/$RID.log"; then
  echo "$(date +%T) $RID infra fault (retry $TRIAL_RETRY)" >> "$OUT/infra_retries.log"
  grep -v "\"run_id\": \"$RID\"" "$OUT/summary.jsonl" > "$OUT/summary.tmp" 2>/dev/null && mv "$OUT/summary.tmp" "$OUT/summary.jsonl"
  if [ $TRIAL_RETRY -lt 2 ]; then TRIAL_RETRY=$((TRIAL_RETRY+1)) exec "$0" "$@"; fi
fi
