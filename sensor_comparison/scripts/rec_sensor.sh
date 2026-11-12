#!/bin/bash
# Usage: rec_sensor.sh <sensor> <speed> <seed> <outdir>   Records real Gazebo GUI + RViz (software rendering, Xvfb) of one study trial.
S=$1; V=$2; SEED=$3; O=$4; mkdir -p $O; R=${S}_v${V}_s${SEED}
source /tmp/env.sh; source /tmp/study/ws/install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1 DISPLAY=:78 QT_QPA_PLATFORM=xcb
for p in "ign gazebo" ruby parameter_bridge static_transform_publisher lifecycle_manager map_server controller_server planner_server smoother_server behavior_server bt_navigator waypoint_follower velocity_smoother robot_state_publisher range_from_scan obstacle_mover study_logger rviz2 "ros2 launch" Xvfb ffmpeg; do pkill -9 -f "$p" 2>/dev/null; done; sleep 2; rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
python3 - "$S" <<'PY'
import sys
s=open('/tmp/study/orig.rviz').read(); sen=sys.argv[1]
s=s.replace("Panels:\n  - Class: rviz_common/Displays\n    Name: Displays\n","Panels: []\n")
s=s.replace("Scale: 60\n      X: 0\n      Y: 0","Scale: 62\n      X: 4.7\n      Y: 0")
if sen.startswith('depth'):
    s=s.replace("""    - Class: rviz_default_plugins/LaserScan
      Name: LaserScan
      Enabled: true
      Size (m): 0.05
      Topic: {Value: /scan, Reliability Policy: Best Effort}""","""    - Class: rviz_default_plugins/PointCloud2
      Name: DepthCloud
      Enabled: true
      Size (m): 0.03
      Topic: {Value: /sensor/depth/points, Reliability Policy: Best Effort}""")
else:
    s=s.replace("Topic: {Value: /scan, Reliability Policy: Best Effort}","Topic: {Value: /sensor/scan, Reliability Policy: Best Effort}")
s=s.replace("""    - Class: rviz_default_plugins/TF
""","""    - Class: rviz_default_plugins/Map
      Name: LocalCostmap
      Enabled: true
      Topic: {Value: /local_costmap/costmap}
    - Class: rviz_default_plugins/TF
""",1)
open('/tmp/study/rec/study_rec.rviz','w').write(s)
PY
Xvfb :78 -screen 0 1600x900x24 >$O/xvfb.log 2>&1 &
sleep 2
ros2 launch throwbot_sensors study.launch.py sensor:=$S speed:=$V seed:=$SEED run_goal:=false rviz:=false out_dir:=$O run_id:=$R > $O/launch_$R.log 2>&1 &
for i in $(seq 1 60); do xdotool search --name "Gazebo" >/dev/null 2>&1 && break; sleep 2; done
sleep 25
rviz2 -d /tmp/study/rec/study_rec.rviz --ros-args -p use_sim_time:=true > $O/rviz_$R.log 2>&1 &
sleep 12
for i in 1 2 3; do
 for w in $(xdotool search --name "Gazebo"); do xdotool windowmove $w 0 0 windowsize $w 900 900; done
 for w in $(xdotool search --name "RViz"); do xdotool windowmove $w 900 0 windowsize $w 700 900; done
 sleep 3
done
ign service -s /gui/move_to/pose --reqtype ignition.msgs.GUICamera --reptype ignition.msgs.Boolean --timeout 3000 --req 'pose: {position: {x: 0, y: 0, z: 13}, orientation: {x: 0, y: 0.7071, z: 0, w: 0.7071}}' > $O/cam_$R.log 2>&1
sleep 5
/usr/bin/ffmpeg -y -loglevel error -f x11grab -video_size 1600x900 -i :78 -frames:v 1 $O/pre_$R.png
/usr/bin/ffmpeg -y -loglevel error -f x11grab -framerate 10 -video_size 1600x900 -i :78 -c:v libx264 -preset ultrafast -crf 26 -pix_fmt yuv420p $O/rec_$R.mp4 </dev/null >$O/ffmpeg_$R.log 2>&1 &
FF=$!
date +%s.%N > $O/rec_start_wall_$R.txt
ros2 run throwbot_sensors study_logger --ros-args -p use_sim_time:=true -p sensor:=$S -p speed:=$V -p seed:=$SEED -p out_dir:=$O -p run_id:=$R > $O/logger_$R.log 2>&1
sleep 4; kill -INT $FF; sleep 2
/usr/bin/ffmpeg -y -loglevel error -ss 0 -i $O/rec_$R.mp4 -frames:v 1 $O/last_$R.png 2>/dev/null
grep -o 'STUDY_RESULT.*' $O/logger_$R.log | cut -c1-600
for p in "ign gazebo" ruby parameter_bridge static_transform_publisher lifecycle_manager map_server controller_server planner_server smoother_server behavior_server bt_navigator waypoint_follower velocity_smoother robot_state_publisher range_from_scan obstacle_mover study_logger rviz2 "ros2 launch" Xvfb; do pkill -9 -f "$p" 2>/dev/null; done
