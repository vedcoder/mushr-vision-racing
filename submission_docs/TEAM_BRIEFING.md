# Team Briefing — read before the presentation

Everything we did, why, where the code lives, the numbers, and the likely
Q&A. Companion docs: [PRESENTATION.md](PRESENTATION.md) (slide guide),
[ARCHITECTURE.md](ARCHITECTURE.md) (system explainer),
[DECISIONS.md](DECISIONS.md) (every design choice + rejected alternative).

## The story arc (tell it in this order)

1. **Platform decision.** Assignment names MuSHR (an Ackermann car). We
   evaluated ROS 2 + Gazebo + TurtleBot3 in parallel but chose **ROS 1
   Noetic + the official MuSHR sim in Docker** — TurtleBot3 is
   differential-drive and cannot demonstrate Ackermann control, and
   infrastructure earns zero rubric points. Docker because Noetic needs
   Ubuntu 20.04 and the team runs macOS/WSL.
2. **Environment engineering.** The stock sim needed: numpy compatibility
   patches (its code predates numpy 1.24 — `np.float` was removed), a
   **synthetic camera** (stock MuSHR sim has none — ours renders the real
   ArUco board textures with true pinhole projection), and **obstacles
   baked into map variants** (the LiDAR raycasts the occupancy grid, so
   obstacles must be in the grid to be seen).
3. **Prototype phase (`testing/`).** Everything was tried small first:
   drive forward → stop at a wall (discovered LiDAR phantom rays) → Pure
   Pursuit → a reactive speed law. These scripts lapped before any "real"
   code existed.
4. **The stack (`race_stack/`).** Formalized into 7 nodes under one rule:
   **everyone publishes opinions; only the arbiter drives.**
5. **Test campaign.** E-stop proof, braking-ramp measurement, dodge
   tests, full runs on all four maps — found the evaluation-B deadlock
   (the war story), led to the urgency-blend fix and a recovery-FSM bug
   fix, and produced the final numbers.
6. **Evidence pipeline.** Every run logs a 20 Hz CSV; one script
   generates the report figures. Tuning was done by instrumenting which
   constraint binds — never by feel.

## File map — which code is where

### race_stack/ — the graded stack (Python/rospy)

| File | What's in it | Key mechanisms |
|---|---|---|
| `scripts/waypoint_manager.py` | Where are we on the track? | vectorized argmin nearest-waypoint; precomputed cumulative-distance table (index → metres); **hysteresis lap counter** (lap counts only on a jump from the last 10% of indices into the first 10%; backwards crossings decrement); publishes the yellow centerline Marker |
| `scripts/path_follower.py` | Steering only | **Pure Pursuit**: carrot 1.2 m ahead → car-frame transform → `steer = atan(2L·sin α / d)` → clamp ±0.34 rad. Publishes `/race/pp_steer`. Never touches speed |
| `scripts/speed_planner.py` | How fast is legal here? | curvature from the CSV's own headings (κ = Δyaw/Δs); `v = v_max/(1+k·κ)`, k = 0.8; **3 m brake-ahead window** (cap = min over next 3 m → brakes *before* corners). Precomputed; runtime is an array lookup |
| `scripts/sign_detector.py` | Reads the speed limits | `cv2.aruco.detectMarkers` (DICT_4X4_50); unknown IDs ignored; <400 px² ignored; largest marker wins; **3 consecutive frames to confirm**; **latch until a different sign confirms**. Caps 10→1.0, 20→1.8, 30→2.5 m/s |
| `scripts/lidar_safety.py` | The reflexes | 720 rays → 3 clearances (±7° centre cone, 11–40° side windows) at the **10th percentile, never the min** (phantom single-ray returns); speed cap = linear ramp 2.5 m → 0.45 m (**the e-stop is the ramp's bottom, not a mode**); dodge = freer side × urgency, capped 0.2 rad |
| `scripts/arbiter.py` | The only node that drives | **`v = min(curve, sign, lidar)`** + rate limits (+2.5/−3.0 m/s²); steering blend `(1−u)·PP + sign(dodge)·u·max` where u = urgency — dodge gets full authority only near stopping distance; **recovery FSM** (RACING→REVERSING→RACING; stuck = no motion for 2 s while commanding motion *or* pinned by the e-stop) |
| `scripts/race_logger.py` | The witness | 20 Hz CSV: pose, commands, min scan, active sign, state, laps, all three caps |
| `scripts/make_plots.py` | CSV → the four report figures + lap times | |
| `launch/race.launch` | Starts all 7 nodes, loads params, `respawn="true"` on every node | |
| `config/params.yaml` | Every tunable number, each with a justification comment | |

### sim/ — environment (ours, but not graded logic)

| File | Purpose |
|---|---|
| `docker-compose.yml` | container + named volume (built workspace survives restarts) |
| `scripts/start_sim.sh` | starts the sim; **auto-applies numpy patches**; **refuses to double-start** (each guard exists because that failure bit us once) |
| `scripts/preflight.sh` | one-command bring-up incl. drivetrain health check — run this before any demo |
| `scripts/sign_camera_sim.py` | synthetic camera: pinhole projection of the real board textures; `_brightness/_blur/_noise_sigma` params for robustness tests |
| `scripts/bake_obstacle_maps.py` | rasterizes obstacle YAMLs into the 5 map variants |
| `patches/` | the three one-word numpy fixes to UW's simulator code |

### Other folders

- `testing/` — the prototype trail (move_forward → move_until_wall →
  follow_waypoints → follow_speed_alpha → safety_test)
- `results/` — official run CSVs behind the report table
- `docs/figures/` — the plots (drag into anything)
- `video/` — the four demo videos

## Numbers to have cold

- **46.5 s** best flying lap (46.6 repeat — 0.1 s consistency), dev map,
  3 laps, **0 collisions everywhere, ever**
- Eval A **63.2 s** — because 60% of the run is in SLOW zones (sign cap
  binding 72% of samples): **"pace is legality"**
- Eval B **68.6 s** — box 9 cm off the centerline, dodged every lap,
  zero recoveries needed
- Vision: **2/2 detections, 0 false transitions** — baseline, −70
  brightness, and 7 px blur
- Thresholds: lookahead 1.2 m · wheelbase 0.33 m · steer ±0.34 rad ·
  stop 0.45 m (= car length + laser offset + one 50 ms tick at 2.5 m/s) ·
  free 2.5 m · dodge trigger 2.2 m, max 0.2 rad · stuck 2 s · reverse
  1.8 s @ 0.6 m/s

## Rapid-fire Q&A (the likely eight)

1. **Why no PID?** Steering: kinematics are known → compute from
   geometry; no gains, no integral state, self-recovers from any
   disturbance. Speed-tracking PID lives in the VESC layer below us,
   where it belongs. We add explicit accel limits at our layer.
2. **Why no neural nets?** Signs are fiducials — deterministic,
   CPU-only, near-zero false positives *by construction* (a candidate
   must decode to a valid dictionary codeword). Proven robust under
   perturbation. The handout endorses this trade.
3. **What if a sign is never visible?** The latched zone persists;
   NORMAL (1.8) before any sign. One dev-layout sign sits ~90° off-axis
   from the racing line — proven to be sightline, not detection (parked
   facing it: confirmed in 3 frames). A wider FOV was tested and
   rejected with data (fewer px/degree lost a previously-detected sign).
4. **Obstacle dead on the line?** The war story: safe deadlock at first
   (bounded dodge loses to Pure Pursuit *by design*) → fixed with
   urgency-weighted authority in the arbiter → retested: passes with
   zero recoveries; the FSM remains as backstop, demonstrated separately
   (on video).
5. **How were thresholds chosen?** Measured. Examples: the ±7° cone was
   ±11° until data showed it grabbing the track walls; k = 0.8 came from
   a probe showing the planner floor-pinned at 1.6; the 2.2 m dodge
   trigger came from a probe showing 40 s/lap crawling behind an
   obstacle at 1.6 m.
6. **Why slower on eval maps?** Point at the caps plot: the green sign
   line sits at 1.0 most of the lap. The car reads its limits off the
   walls — nothing is keyed to waypoint numbers, which is exactly what
   hidden-layout evaluation demands.
7. **What breaks it?** Honest list: localization drift untested
   (ground-truth odom, switchable by parameter); forward-only safety
   cones; no racing-line optimization. Naming limitations is the
   credibility move.
8. **Who did what / AI use?** Contribution statement is in the report;
   development was pair-programmed with an AI assistant, disclosed, with
   the commit history transparently co-authored — and every design
   decision is one the team can defend.

## Stage kit

- RUNBOOK.md open in a tab
- Terminal ready with: `bash sim/scripts/preflight.sh`
- The recovery-demo teleport command ready in a second terminal (in the
  RUNBOOK, "Live demo" section)
- Videos downloaded locally as backup for the Drive links
