#!/bin/bash
# One-command bring-up + verification, from the HOST (Mac/WSL terminal):
#   bash sim/scripts/preflight.sh
# Starts container + sim + stack, health-checks the drivetrain, verifies
# every /race topic is flowing. Safe to re-run: refuses nothing, checks
# everything. Exit 0 = ready to demo.
set -e
cd "$(dirname "$0")/../"    # sim/

echo "[1/5] container..."
docker compose start 2>/dev/null || docker compose up -d

echo "[2/5] simulator..."
docker exec mushr_sim bash -c 'source /opt/ros/noetic/setup.bash; rostopic list 2>/dev/null | grep -q /car/scan' \
  || docker exec -d mushr_sim bash /assignment/sim/scripts/start_sim.sh track_development development
docker exec mushr_sim bash -c 'source /opt/ros/noetic/setup.bash
for i in $(seq 1 50); do rosnode list 2>/dev/null | grep -q ackermann_to_vesc && exit 0; sleep 2; done
echo "SIM FAILED TO START"; exit 1'

echo "[3/5] drivetrain health check..."
docker exec mushr_sim bash -c 'source /opt/ros/noetic/setup.bash
P0=$(rostopic echo -n1 /mushr_sim/car/odom/pose/pose/position/x | head -1)
timeout 3 rostopic pub -r 10 /car/mux/ackermann_cmd_mux/input/navigation ackermann_msgs/AckermannDriveStamped "{drive: {speed: 0.5}}" >/dev/null 2>&1
P1=$(rostopic echo -n1 /mushr_sim/car/odom/pose/pose/position/x | head -1)
python3 -c "import sys; a,b=float(\"$P0\"),float(\"$P1\"); sys.exit(0 if abs(b-a)>0.3 else 1)" \
  && echo "  car moves: $P0 -> $P1" \
  || { echo "  DRIVETRAIN DEAD ($P0 -> $P1) — restart the sim:"; echo "  docker exec mushr_sim pkill -f ros && re-run preflight"; exit 1; }'

echo "[4/5] racing stack..."
docker exec mushr_sim bash -c 'source /opt/ros/noetic/setup.bash; rostopic list 2>/dev/null | grep -q /race/target_speed' \
  || { docker exec -d mushr_sim bash -c 'source /opt/ros/noetic/setup.bash && source /root/catkin_ws/devel/setup.bash && roslaunch race_stack race.launch > /assignment/sim/logs/race_stack.log 2>&1'; sleep 10; }

echo "[5/5] verifying all /race topics flow..."
docker exec mushr_sim bash -c 'source /opt/ros/noetic/setup.bash
for t in /race/target_speed /race/nearest_waypoint /race/curve_speed_cap /race/lidar_speed_cap; do
  timeout 4 rostopic echo -n1 $t >/dev/null 2>&1 && echo "  ok $t" || { echo "  MISSING $t"; exit 1; }
done'

echo "PREFLIGHT PASSED — car is racing. Foxglove: ws://localhost:9090"
