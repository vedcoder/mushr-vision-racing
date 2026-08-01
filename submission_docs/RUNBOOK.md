# Runbook — exact commands to start, run, and stop everything

Copy-paste ready. All `docker exec` commands work from any Mac/WSL
terminal in this repo. During the presentation, keep this file open.

## Cold start (laptop just booted) — ~2 min

```bash
open -a Docker                # macOS; on Windows start Docker Desktop
# wait until Docker Desktop says "running" (whale icon steady)

cd sim
docker compose up -d          # start (or recreate) the container

# start the simulator on the development track + sign layout
docker exec -d mushr_sim bash /assignment/sim/scripts/start_sim.sh track_development development

# start our racing stack (all 6 nodes)
docker exec -d mushr_sim bash -c 'source /opt/ros/noetic/setup.bash && source /root/catkin_ws/devel/setup.bash && roslaunch race_stack race.launch'
```

Then open Foxglove Studio → Open connection → **Rosbridge (ROS 1 & 2)** →
`ws://localhost:9090`.

The car starts driving as soon as the stack is up (arbiter commands
immediately). To hold it still until you're ready, start the stack only
when asked to "make it go".

## Put the car on the start line

```bash
docker exec mushr_sim bash -c 'source /opt/ros/noetic/setup.bash; rostopic pub -1 /mushr_sim/reposition geometry_msgs/PoseStamped "{header: {frame_id: map}, pose: {position: {x: 3.4, y: 4.0}, orientation: {w: 1.0}}}"'
```

## Stop / start JUST the racing stack (car halts ~0.2 s after stop)

```bash
docker exec mushr_sim pkill -f race.launch      # stop our 6 nodes
# restart:
docker exec -d mushr_sim bash -c 'source /opt/ros/noetic/setup.bash && source /root/catkin_ws/devel/setup.bash && roslaunch race_stack race.launch'
```

## Switch maps (e.g. to an evaluation set)

```bash
docker exec mushr_sim pkill -f ros; sleep 3
docker exec -d mushr_sim bash /assignment/sim/scripts/start_sim.sh track_evaluation_A evaluation_A
# wait ~20 s, then restart the racing stack (command above)
```

Maps: `track_clean`, `track_development`, `track_evaluation_A/B/C` with
sign layouts `development`, `evaluation_A/B/C`.

## Full shutdown

```bash
docker exec mushr_sim pkill -f ros              # stop all ROS processes
cd sim && docker compose stop                   # stop container (state kept)
# optionally quit Docker Desktop and Foxglove
```

## Live demo: forced recovery (the showpiece)

With the stack running on `track_development`, teleport the car 30 cm
from the box on the straight — it e-stops, waits 2 s, reverses with a
nose-swing, dodges around, and resumes racing on its own:

```bash
docker exec mushr_sim bash -c 'source /opt/ros/noetic/setup.bash; rostopic pub -1 /mushr_sim/reposition geometry_msgs/PoseStamped "{header: {frame_id: map}, pose: {position: {x: 12.6, y: 3.65}, orientation: {w: 1.0}}}"'
```

Narrate from `rostopic echo /race/state` in a second terminal:
RACING → REVERSING → RACING.

## Useful live checks

```bash
# inside `docker exec -it mushr_sim bash` (ROS env auto-loads):
rostopic echo /race/lap_count      # laps
rostopic echo /race/state          # RACING / REVERSING
rostopic echo /race/active_sign    # current speed-zone sign
rostopic list | grep /race/        # all our topics (should be 10)
```

## Health check BEFORE any official run (10 seconds)

The sim can boot with a dead drivetrain (double-start race kills
ackermann_to_vesc). Verify the car physically moves before trusting a run:

```bash
docker exec mushr_sim bash -c 'source /opt/ros/noetic/setup.bash; P0=$(rostopic echo -n1 /mushr_sim/car/odom/pose/pose/position/x | head -1); timeout 3 rostopic pub -r 10 /car/mux/ackermann_cmd_mux/input/navigation ackermann_msgs/AckermannDriveStamped "{drive: {speed: 0.5}}" >/dev/null 2>&1; P1=$(rostopic echo -n1 /mushr_sim/car/odom/pose/pose/position/x | head -1); echo "x: $P0 -> $P1"'
```

If x does not change: restart the sim (pkill -f ros, start_sim again).

## If something is wedged

- Foxglove won't connect → is rosbridge up? `nc -z localhost 9090`.
  Rosbridge starts with the sim, not the stack.
- Sim slow / laps crawling → Docker Desktop degraded: quit Docker fully,
  relaunch, cold-start again (known issue, documented in BUILDLOG).
- Container name conflict on `compose up` → `docker rm -f mushr_sim`,
  retry.
