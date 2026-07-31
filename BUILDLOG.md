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
| 1 | Waypoint manager: nearest waypoint, progress, lap counter | Reliable completion + lap counting (20%) | 🔲 |
| 2 | Pure Pursuit path follower | Path following (20%) | 🔲 |
| 3 | Curvature-aware speed planner | Speed control (part of 20%) | 🔲 |
| 4 | ArUco sign detector + speed-zone state | Vision (17%) | 🔲 |
| 5 | LiDAR safety: clearance speed limit, e-stop, avoidance | LiDAR safety (18%) | 🔲 |
| 6 | Command arbiter (min-speed rule) + recovery FSM | Recovery (10%) | 🔲 |
| 7 | CSV logger + plot generation | Analysis & reproducibility (10%) | 🔲 |

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

## Step 1 — Waypoint manager (planned)

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
