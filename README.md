# MuSHR Vision-Aware Autonomous Racing — Team Repo

Our team's stack for the 5-day MuSHR Vision-Aware Autonomous Racing
Challenge: an autonomous racing system that must complete three consecutive
laps, brake for corners, avoid LiDAR-visible obstacles, read ArUco speed
signs from a forward camera, and recover from failures — all in simulation.

The full student handout is in
[`docs/mushr_vision_racing_assignment.pdf`](docs/mushr_vision_racing_assignment.pdf).

## Why this setup (decisions & rationale)

**ROS 1 Noetic + the official MuSHR simulator, not ROS 2/Gazebo/TurtleBot3.**
The assignment requires an Ackermann (car-like) vehicle and names MuSHR as
the platform. TurtleBot3 is differential-drive — using it would mean 1–2
days of Gazebo work to build a car model before any graded work starts.
The official MuSHR sim gives us the correct vehicle, LiDAR, and drive
interface out of the box, so all of our ~100 team-hours go into the parts
the rubric actually scores (control 20%, LiDAR safety 18%, vision 17%, …).
ROS 1 code cannot be mixed with ROS 2 code, so this choice is binding for
the whole team.

**Docker, because ROS 1 needs Ubuntu 20.04.** ROS Noetic does not run
natively on macOS/Windows. Docker is also MuSHR's officially supported
install path. The container (`mushr/mushr:x86_64`) runs under Rosetta on
Apple Silicon; it has been verified end-to-end on an M-series Mac (LiDAR at
10 Hz, ground-truth odom at 20 Hz, drive commands moving the car).

**Foxglove Studio instead of RViz.** No X11 forwarding headaches on
macOS/Windows: the sim exposes rosbridge on `ws://localhost:9090` and
Foxglove (native desktop app) renders the map, scans, camera, and teleop.

**Obstacles are baked into map variants.** The sim's LiDAR raycasts against
the occupancy grid, so `sim/scripts/bake_obstacle_maps.py` rasterizes each
obstacle set from `config/obstacle_sets.yaml` into its own map
(`track_clean`, `track_development`, `track_evaluation_A/B/C`). This
guarantees obstacles are LiDAR-visible, as the handout requires.

**The camera is synthetic.** Stock mushr_sim has no RGB camera; the handout
explicitly expects an instructor/team adapter here.
`sim/scripts/sign_camera_sim.py` renders the real ArUco board textures from
`signs/` at the poses in `config/vision_sign_layouts.yaml` with a proper
homography (pinhole) projection, and publishes
`/camera/front/image_raw` at ~15 Hz. Detection difficulty scales
realistically with distance and viewing angle, so an OpenCV ArUco pipeline
behaves like it would on a real feed.

**Upstream numpy patches.** The MuSHR sim source predates numpy 1.24 and
uses the removed `np.float`/`np.int` aliases; the LiDAR node
(`fake_urg.py`, `mushr_sim.py`) and the state publisher
(`racecar_state.py`, patched copy in `sim/patches/`) crash without the
one-word fixes (`np.float` → `float`). These are simulator bug-fixes only —
no assignment logic lives there.

## Quickstart

Prereqs: Docker Desktop, [Foxglove Studio](https://foxglove.dev/download).

```bash
cd sim
docker compose up -d

# first time only: build the workspace and apply the numpy patches
docker exec mushr_sim bash -c 'source /opt/ros/noetic/setup.bash && cd /root/catkin_ws && catkin build'
docker cp patches/racecar_state.py mushr_sim:/root/catkin_ws/src/mushr/mushr_base/mushr_base/src/mushr_base/racecar_state.py
# (fake_urg.py / mushr_sim.py need the same np.float->float, np.int->int edits)

# start the sim on the development layout
docker exec -d mushr_sim bash /assignment/sim/scripts/start_sim.sh track_development development
```

Then open Foxglove → Open connection → **Rosbridge (ROS 1 & 2)** →
`ws://localhost:9090`. See [`sim/README.md`](sim/README.md) for panels,
teleop, teleporting the car, and all day-to-day commands.

## Topic contract

| Topic | Type | Direction |
|---|---|---|
| `/car/scan` | `sensor_msgs/LaserScan` | in (10 Hz) |
| `/camera/front/image_raw` | `sensor_msgs/Image` | in (~15 Hz) |
| `/car/odom`, `/mushr_sim/car/odom` | `nav_msgs/Odometry` | in |
| `/car/mux/ackermann_cmd_mux/input/navigation` | `AckermannDriveStamped` | **out (our controller)** |

## Repo layout

| Path | What it is |
|---|---|
| `docs/` | Assignment handout (PDF + LaTeX source) |
| `track/` | Occupancy-grid map, centerline waypoints, preview (instructor-provided) |
| `config/` | Obstacle sets, sign layouts, track sectors (instructor-provided) |
| `signs/` | ArUco sign-board textures (instructor-provided) |
| `scripts/` | Instructor asset generator |
| `sim/` | Our simulation environment: Docker setup, launch scripts, camera sim, map baking, patches |

Instructor-provided assets are unmodified. Team-authored code lives (and
will grow) under `sim/` and, soon, our ROS package(s) for control, vision,
safety, and recovery.
