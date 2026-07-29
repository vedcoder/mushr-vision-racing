# Sim setup (MuSHR in Docker + Foxglove on macOS)

Everything runs inside the `mushr_sim` container (image `mushr/mushr:x86_64`
under Rosetta). The assignment folder is mounted at `/assignment`.

## Start / stop

```bash
cd sim
docker compose up -d                    # start container
docker exec -d mushr_sim bash /assignment/sim/scripts/start_sim.sh track_development development
docker compose down                     # stop everything
```

`start_sim.sh [map] [sign_layout]` — maps: `track_clean`,
`track_development`, `track_evaluation_A/B/C` (obstacles are baked into the
occupancy grid so LiDAR sees them). Sign layouts come from
`config/vision_sign_layouts.yaml`.

## Watch what's going on (Foxglove)

Open Foxglove Studio → *Open connection* → **Rosbridge** →
`ws://localhost:9090`. Useful panels:

- **3D**: add `/map`, `/car/scan`, TF; shows the car on the track
- **Image**: `/camera/front/image_raw` — the synthetic sign camera
- **Teleop**: publishes to `/car/mux/ackermann_cmd_mux/input/teleop` to drive manually
- **Raw messages**: `/mushr_sim/car/odom` for ground-truth pose

## Topic contract (matches the handout)

| Topic | Type | Notes |
|---|---|---|
| `/car/scan` | sensor_msgs/LaserScan | 10 Hz simulated LiDAR |
| `/camera/front/image_raw` | sensor_msgs/Image | ~15 Hz, renders ArUco boards |
| `/mushr_sim/car/odom` | nav_msgs/Odometry | ground truth, 20 Hz |
| `/car/odom` | nav_msgs/Odometry | VESC odom, active once driving |
| `/car/mux/ackermann_cmd_mux/input/navigation` | AckermannDriveStamped | your controller publishes here |

## Handy commands

```bash
# shell inside the container (ROS env auto-sourced)
docker exec -it mushr_sim bash

# list topics / check rates
docker exec mushr_sim bash -c 'source /opt/ros/noetic/setup.bash; rostopic list'
docker exec mushr_sim bash -c 'source /opt/ros/noetic/setup.bash; rostopic hz /car/scan'

# teleport the car back to the start line
docker exec mushr_sim bash -c 'source /opt/ros/noetic/setup.bash; rostopic pub -1 /mushr_sim/reposition geometry_msgs/PoseStamped "{header: {frame_id: map}, pose: {position: {x: 3.4, y: 4.0}, orientation: {w: 1.0}}}"'

# logs
tail -f sim/logs/mushr_sim.log sim/logs/sign_camera.log
```

## Notes / gotchas

- The catkin workspace lives in a Docker volume; it was built once with
  `catkin build`. If the container is recreated with `docker compose down -v`
  the build (and the numpy patch below) must be redone.
- `mushr_sim` source was patched in-container: `np.float`/`np.int` →
  `float`/`int` (removed in numpy ≥1.24; the LiDAR node crashed without it).
- The sign camera is synthetic (`scripts/sign_camera_sim.py`) because stock
  mushr_sim has no RGB camera. It renders the boards in `signs/` at the poses
  from the chosen layout with a proper homography warp, so OpenCV ArUco
  detection behaves realistically.
