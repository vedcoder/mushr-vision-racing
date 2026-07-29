#!/bin/bash
# Runs INSIDE the mushr container. Brings up the MuSHR sim on the assignment
# track plus the synthetic sign camera. teleop.launch already includes
# map_server (map from $MAP) and rosbridge on :9090 for Foxglove.
#   usage: start_sim.sh [map_name] [sign_layout]
#   e.g.   start_sim.sh track_development development
set -e
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash

MAP_NAME=${1:-track_development}
LAYOUT=${2:-development}
export MAP=/assignment/sim/maps/${MAP_NAME}.yaml
if [ ! -f "$MAP" ]; then
  echo "Baking obstacle map variants..."
  python3 /assignment/sim/scripts/bake_obstacle_maps.py
fi

mkdir -p /assignment/sim/logs

# Start at the start/finish line (from config/track_sectors.yaml)
roslaunch mushr_sim teleop.launch \
  initial_x:=3.4 initial_y:=4.0 initial_theta:=0.0 \
  > /assignment/sim/logs/mushr_sim.log 2>&1 &
SIM_PID=$!

# Wait for the sim to come up
for i in $(seq 1 60); do
  rostopic list >/dev/null 2>&1 && break
  sleep 1
done
sleep 5

# Synthetic forward camera rendering the ArUco sign boards
python3 /assignment/sim/scripts/sign_camera_sim.py \
  _assignment_root:=/assignment _layout:=${LAYOUT} \
  > /assignment/sim/logs/sign_camera.log 2>&1 &

sleep 3
echo "==== topics ===="
rostopic list
echo "==== sim running (map=${MAP_NAME}, signs=${LAYOUT}) ===="
wait $SIM_PID
