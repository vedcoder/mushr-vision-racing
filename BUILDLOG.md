# Build Log — race stack

This document grows alongside the code. Every step gets a section here at
the same time as the code lands: **what** was built, **why** it's designed
that way, **how the algorithm works** in plain language, and **how to
verify it** yourself in the sim. Read it commit-by-commit and you should be
able to explain (and defend in the presentation) every part of the stack.

Status legend: 🔲 planned · 🚧 in progress · ✅ done

## The plan (maps to the grading rubric)

| Step | What | Rubric weight it targets | Status |
|---|---|---|---|
| 1 | Waypoint manager: nearest waypoint, progress, lap counter | Reliable completion + lap counting (20%) | ✅ |
| 2 | Pure Pursuit path follower | Path following (20%) | ✅ |
| 3 | Curvature-aware speed planner | Speed control (part of 20%) | ✅ |
| 4 | ArUco sign detector + speed-zone state | Vision (17%) | ✅ |
| 5 | LiDAR safety: clearance speed limit, e-stop, avoidance | LiDAR safety (18%) | ✅ |
| 6 | Command arbiter (min-speed rule) + recovery FSM | Recovery (10%) | ✅ |
| 7 | CSV logger + plot generation | Analysis & reproducibility (10%) | ✅ |

All code will live in a single catkin package, `race_stack/`, at the repo
root, symlinked into the container's workspace. One node per concern, so
four people can work in parallel without merge conflicts.

Interfaces between our own nodes (agreed now, so work can be split):

| Topic | Type | Published by | Meaning |
|---|---|---|---|
| `/race/nearest_waypoint` | Int32 | waypoint manager | index of closest centerline point |
| `/race/progress_m` | Float32 | waypoint manager | metres travelled along centerline |
| `/race/lap_count` | Int32 | waypoint manager | completed laps |
| `/race/active_sign` | Int32 | sign detector | last confirmed marker ID (10/20/30) |
| `/race/target_speed` | Float32 | arbiter | final commanded speed |
| `/race/state` | String | recovery FSM | RACING / STUCK / REVERSING / … |

The single output to the car is
`/car/mux/ackermann_cmd_mux/input/navigation` (AckermannDriveStamped),
published **only** by the arbiter — no other node commands the car
directly. That makes the min() rule enforceable and the mux input
deterministic.

---

## Step 1 — Waypoint manager ✅ (implemented in race_stack/)

**Landed as designed below, plus one addition:** the node also publishes
the centerline as a latched `visualization_msgs/Marker` on
`/race/waypoints_viz` — enable it in Foxglove's 3D panel to see the yellow
line the car follows. Package skeleton: `race_stack/` at the repo root,
symlinked into the container workspace (`ln -s /assignment/race_stack
/root/catkin_ws/src/race_stack`, then `catkin build race_stack` once).

**Verified:** with the sandbox Pure Pursuit follower driving,
`/race/lap_count` went 0 → 1 exactly at the finish line, 68 s after the
start — and nudging across the line back/forward does not inflate the
count (hysteresis working).

**Run it:**
```bash
rosrun race_stack waypoint_manager.py
rostopic echo /race/progress_m      # watch progress climb as the car drives
```

### Original design (for reference)

**What.** A node that loads `track/centerline_waypoints.csv` (337 points:
id, x, y, yaw), listens to odometry, and continuously publishes: the index
of the nearest waypoint, progress along the track in metres, and the lap
count.

**Why first.** Every later node needs "where are we on the track?":
Pure Pursuit picks its lookahead from the nearest index, the speed planner
looks up upcoming curvature from it, scoring needs lap times. It is also
the smallest possible complete rospy node — subscriber, publisher, a bit of
numpy — so it's the right first exposure to the codebase for everyone.

**How it will work.**
- Load the CSV once into an N×3 numpy array; precompute the cumulative
  distance along the centerline (running sum of segment lengths), which
  turns "waypoint index" into "metres of progress" for free.
- On every odometry message: nearest waypoint = argmin of squared distance
  from the car to all 337 points (337 points × 20 Hz is tiny; no k-d tree
  needed — a deliberate simplicity choice).
- Lap detection with hysteresis: a lap counts only when the nearest index
  jumps from the last ~10% of the track into the first ~10% (i.e. the car
  actually crossed the start line going forward). Crossing backwards
  decrements. This prevents false laps from noise when the car sits near
  the line, and prevents "wiggle across the line" from counting twice.
- Odometry source is a parameter. Default: ground truth
  (`/mushr_sim/car/odom`) during development; switchable without code
  changes if evaluation requires the estimated pose instead.

**How we'll verify.** Start the sim, run the node, drive manually with
Foxglove teleop. Watch `/race/nearest_waypoint` count up as the car moves,
and `/race/lap_count` tick exactly once per completed lap (and not tick
when nudging back and forth across the start line). Deliverable artifact:
a short screen capture + the topic echo.

---

## Sandbox prototypes (pre-Step-1 learning experiments) ✅

Three throwaway scripts in `sandbox/` — not part of the final package, but
they prototype the core loops and are worth reading in order:

1. **`move_forward.py`** — the "hello world": stream
   AckermannDriveStamped at 10 Hz to the navigation mux input. Lesson: the
   mux stops the car ~0.2 s after you stop publishing; commands are a
   stream, not one-shots.
2. **`move_until_wall.py`** — LiDAR reflex: watch a ±20° forward cone of
   `/car/scan`, stop when clearance < 0.6 m. Lesson learned the hard way:
   the (realistic) laser noise model produces phantom short readings on
   single rays — the first version stopped for a ghost. Fix: require ≥5
   adjacent rays to agree before believing an obstacle. Never act on a
   single sensor reading.
3. **`follow_speed_alpha.py`** (and constant-speed `follow_waypoints.py`) —
   Pure Pursuit chasing a carrot 1.2 m ahead on the centerline, plus a
   reactive speed law `v = 2.0/(1+4|alpha|)` clipped to [0.8, 2.0].
   **Result: full autonomous laps, ~64 s, zero wall contact, lateral error
   ≤ 0.24 m.** Known flaw, visible in the logs: alpha only grows once the
   car is already at the corner, so braking happens late — at higher V_MAX
   this will run wide. The real Step-3 planner fixes this by reading track
   curvature *ahead* of the car instead of reacting to the present.

These become Steps 2/3/5 properly: split into nodes, arbiter enforcing
min(), parameters in config files, logging to CSV.

*Next entries will be added when each step's code lands.*

**Build order note:** steps are being done in dependency order, not list
order: 1 → 4 → 5 → (2+3+6 together: formalize follower + curvature planner
+ arbiter) → 7. Reason: the arbiter's `min(v_curve, v_sign, v_lidar)`
defines how the follower must be restructured, so the two cap producers
(signs, LiDAR) land first and the restructuring happens exactly once. The
sandbox follower stands in for Steps 2–3 until then.

## Step 4 — ArUco sign detector ✅ (race_stack/scripts/sign_detector.py)

**What.** Reads `/camera/front/image_raw`, detects DICT_4X4_50 markers with
OpenCV, and maintains the *active speed sign*. Publishes latched
`/race/active_sign` (10/20/30, −1 before any sign) and
`/race/sign_speed_cap` (1.0/1.8/2.5 m/s; default 1.8 before any sign).

**Design decisions (the vision requirements in the handout):**
- *Temporal confirmation:* a marker must appear in 3 consecutive frames to
  become active — a single-frame glitch can never flip the speed zone.
- *Latching:* the active sign persists after the board leaves view, until a
  different sign is confirmed (per the rules).
- *Largest-marker-wins:* if two signs are visible, the nearer (bigger in
  pixels) one is taken; markers under 400 px² are ignored as too far.
- No cv_bridge: raw `Image.data` → numpy reshape (bgr8), one less
  dependency.

**Verified:** with the sandbox follower lapping the development layout, the
log shows `20 → 30 → 20 → 30` (NORMAL and BOOST confirmed every lap, caps
1.8/2.5 correct, latch holding in between). Detector confirmed ID 10 in
<3 frames when the car was parked facing the SLOW board.

**Known issue (dev layout):** the SLOW sign at (24.4, 7.5) never enters
frontal view from the racing line — it sits ~90° off the camera axis as
the car passes. Verified it *detects* when in view, so this is sign
placement/sightline, not detection. A 90° FOV camera was tried and
reverted: fewer pixels per degree lost the BOOST sign without gaining
SLOW. Watch for this in evaluation layouts; if a required sign is
missed, the first knob is camera resolution, not FOV.

**Run it:**
```bash
rosrun race_stack sign_detector.py
rostopic echo /race/active_sign     # -1, then 20/30/10 as boards are passed
```

## Step 5 — LiDAR safety ✅ (race_stack/scripts/lidar_safety.py)

**What.** One 360° scan → three numbers (robust clearance in a ±7° centre
cone and 11–40° left/right windows) → two outputs: `/race/lidar_speed_cap`
(linear ramp: no limit above 2.5 m clearance, zero at 0.45 m — the e-stop
is the bottom of the ramp, not a separate mode) and `/race/dodge_steer`
(bias toward the freer side, scaled by urgency, max 0.2 rad so Pure
Pursuit can always out-vote it and pull back to the line).

**Design notes for the report:**
- Every clearance is a 10th-percentile, never a raw min — the sim's laser
  noise produces phantom single-ray shorts (see sandbox notes).
- STOP=0.45 m justified by car length + laser offset + worst-case one-tick
  travel at 2.5 m/s. FREE=2.5 m gives gentle braking from top legal speed.
- Centre cone was ±11° initially; on this 2.3 m-wide track it grazed the
  walls and bound the cap at ~1.9 on open straights. Fixed at ±7° —
  measured, not guessed (both log excerpts kept for the report).
- Geometry fact found while testing: the corner barrel intrudes on the
  centerline (line passes 0.36 m from its centre, r=0.28, car half-width
  0.14 — negative margin). It is the real dodge test case, near wp 125.

**Verified:** full autonomous lap on track_development with follower +
signs + safety all active: lap counted, cap quiet except 28 events, dodge
fired with correct direction at obstacle encounters (e.g. centre 0.97 m,
right 0.90 m → bias +0.11 left; caps dipped to 0.40–0.64 during
encounters), car resumed racing after each.

**Run it:**
```bash
rosrun race_stack lidar_safety.py
rostopic echo /race/lidar_speed_cap
```

### Step 5 test campaign (results)

| Test | Result |
|---|---|
| E-stop: straight-driver obeying only the cap, aimed at the box | **PASS** — cap ramped 2.30→0.00 smoothly, car halted 0.4 m short, held stop (`sandbox/safety_test.py`, table in logs) |
| Braking-ramp shape | **PASS** — linear descent as designed, one benign noise blip re-clamped immediately |
| Barrel head-on with full stack | **PASS** — dodge +0.11 with cap 0.62 at the barrel, then threaded the gate pair, no contact |
| Evaluation A (unseen map + signs) | **PASS** — 2+ laps, 5 sign confirmations, 7 dodges, no contact |
| Evaluation B | **SAFE-DEADLOCK** — box_B1 sits 0.09 m off centerline (wp 294): e-stop prevented collision (4 cm final clearance, zero contact) but car froze forever: Pure Pursuit (0.34 rad authority) out-votes the dodge (0.2 by design) and keeps aiming through the box. Root cause understood; fix = recovery FSM + reducing PP authority while dodge urgency is high (planned Step 6). |
| Evaluation C | **PASS** (on rerun with healthy Docker) — 2+ laps, no contact, no stall; 160 control-loop log lines vs 45 in the first attempt, confirming the original failure was Docker starving the sim, not the stack. Only SLOW is sightline-visible in this layout → run legally capped at 1.0 m/s (78 s laps). |

## Steps 2+3+6 — formalization: follower, planner, arbiter ✅

The sandbox retires. Six nodes, one launch file
(`roslaunch race_stack race.launch`), tuning in `config/params.yaml`:

- **path_follower.py (Step 2):** Pure Pursuit, steering opinion only on
  `/race/pp_steer`. Parameters from the config, not constants.
- **speed_planner.py (Step 3):** curvature from the CSV's own headings
  (kappa = yaw change per metre), the handout's law
  v = v_max/(1+k|kappa|), and a 3 m brake-ahead window (cap at wp i =
  min allowed speed over the next 3 m) — fixes the reactive alpha law's
  measured late-braking flaw. All precomputed; runtime is a lookup.
- **arbiter.py (Step 6):** the only node that drives.
  v = min(curve, sign, lidar) with accel/decel rate limiting;
  steering = (1-u)·pp + sign(dodge)·u·max_steer where u = dodge urgency —
  at full urgency the dodge has FULL authority (the evaluation-B fix).
  Recovery FSM: RACING → REVERSING (back out, nose toward free space) →
  RACING, published on `/race/state`.

**Verified:**
- Dev map, full stack via race.launch: 2 laps in 150 s, no contact.
- Evaluation B (the old deadlock): **2 laps, zero recoveries needed** —
  the urgency blend steers around the on-line box on its own.
- Forced deadlock (teleported nose-to-box, e-stop pinned): stuck detector
  fired at 2 s, `RACING → REVERSING → RACING`, car backed out, dodged the
  box, completed the lap.
- **Bug found by testing:** the stuck detector originally required
  cmd_v > 0.1, but a full e-stop sets cmd_v = 0 — recovery was unreachable
  in exactly the deepest deadlock. Fixed: also trigger when the lidar cap
  pins the car (< 0.3). Kept here because it's a good war story.

## Step 7 — race logger + plots ✅

`race_logger.py` runs with the stack (race.launch, `run_name:=<name>`),
writing 20 Hz CSV rows with every submission-checklist column (time, pose,
commanded speed/steering, min LiDAR range, active sign, state, lap,
progress, lateral error, all three caps). `make_plots.py <csv>` produces
the four required figures: speed-vs-caps (the arbiter picture),
speed profile by track position, tracking error, obstacle clearance —
plus lap times.

**Official 3-lap run on track_development:** 3 laps, flying laps
**48.9 s and 48.8 s** (0.1 s repeatability), zero contact.

**The logger paid for itself on its first outing:** run #1 recorded the
car frozen at the start line while commanding 1.1 m/s — the CSV proved
the stack was fine and the *sim's* drivetrain was dead
(ackermann_to_vesc killed by a double sim start: "new node registered
with same name"). New rule in the RUNBOOK: before any official run, do a
2-second manual drive test; if the car doesn't move, restart the sim.

**Lesson worth presenting:** the failures found are architectural, not
bugs — a bounded dodge *cannot* beat the path follower for obstacles dead
on the racing line, by deliberate design (the same bound that keeps the
dodge from steering the car off-track). The system chose safety
(stop, no contact) over progress; restoring progress is the recovery
FSM's job. This is exactly the failure-mode analysis the report asks for.
