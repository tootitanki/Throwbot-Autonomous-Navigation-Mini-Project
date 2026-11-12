#!/bin/bash
# Launch one depth trial and concurrently record (a) Gazebo world pose of robot+box, (b) gz-side vs ROS-side depth cloud publication stamps.
source /tmp/env.sh; source /tmp/study/ws/install/setup.bash; export LIBGL_ALWAYS_SOFTWARE=1
D=/tmp/study/diag; rm -rf $D; mkdir -p $D
SENS=${1:-depth}
cd /tmp/study/ws; (setsid ./run_trial.sh $SENS 0.5 ${2:-98} $D > $D/trial.out 2>&1 &)
TOP=/sensor/depth/points; case $SENS in depth*) ;; *) TOP=/sensor/scan;; esac
for i in $(seq 1 60); do ign topic -l 2>/dev/null | grep -q "$TOP" && break; sleep 1; done
sleep 12
(timeout 45 ign topic -e -t /world/test_room/dynamic_pose/info > $D/pose_stream.txt 2>&1 &)
(timeout 25 ign topic -e -t $TOP 2>&1 | grep -E '^\s+(sec|nsec):' | head -400 > $D/gz_stamps.txt &)
(timeout 25 ros2 topic hz $TOP --window 20 > $D/ros_hz.txt 2>&1 &)
(timeout 25 ign topic -e -t /clock 2>&1 | grep -E 'sec:' | head -20 > $D/clock.txt &)
for i in $(seq 1 70); do grep -q STUDY_RESULT $D/trial.out && break; sleep 2; done
sleep 3; echo done
