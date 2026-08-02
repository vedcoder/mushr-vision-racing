# Decision Log — every major choice, the alternatives, and why

The tech stack and design decisions in one place, each with the
alternative we rejected and the reason. Companion to
[ARCHITECTURE.md](ARCHITECTURE.md) (how the system works) and
[BUILDLOG.md](BUILDLOG.md) (chronological history with measurements).

## 1. Tech stack

### ROS 1 Noetic — not ROS 2
The team initially evaluated both (a ROS 2 Humble + Gazebo + TurtleBot3
environment was set up in parallel during the platform decision). We chose
ROS 1 because the assignment names MuSHR, whose official simulator is
ROS 1; the alternative required building an Ackermann vehicle in Gazebo
before any graded work could start (TurtleBot3 is differential-drive — it
cannot demonstrate Ackermann control at all). With ~5 days total and zero
rubric points for infrastructure, we bought the platform and spent the
time on the graded stack. ROS 1's EOL status is a real cost we accepted
knowingly.

### Official MuSHR simulator in Docker — not native install or custom sim
ROS Noetic requires Ubuntu 20.04; the team runs macOS and Windows/WSL.
Docker is also MuSHR's officially supported install path. The container
plus a named volume gives every teammate the identical, reproducible
environment (one compose file, one start script). Costs accepted: x86
emulation on Apple Silicon, and Docker Desktop's occasional degradation —
mitigated by the pre-run health check in the RUNBOOK.

### Foxglove Studio over RViz
RViz needs X11 forwarding on macOS/Windows (fragile, slow). The sim
already ships rosbridge; Foxglove connects over a websocket as a native
desktop app on every teammate's OS. Zero display-forwarding debugging all
project.

### Python (rospy) for the entire stack — no C++
At 20 Hz control with 337 waypoints and 720 rays, numpy vectorization is
ample (measured: callbacks run in microseconds). Python maximized
iteration speed — the sandbox-prototype-to-node pipeline that produced the
whole stack in two days is not realistic in C++ at our team's fluency.

### Classical vision (OpenCV ArUco) — no deep learning
The signs are ArUco fiducials: binary-coded patterns whose detection is
deterministic, fast, CPU-only, and near-immune to false positives by
construction (a candidate must decode to a valid dictionary codeword).
A neural detector would add training data requirements, GPU dependency,
and un-explainable failures to solve a problem that is already solved.
Verified: 2/2 detections, 0 false transitions under darkening and blur.
The handout explicitly endorses this trade.

### Synthetic camera (custom node) — because stock mushr_sim has none
The official sim has no RGB camera. Our `sign_camera_sim.py` renders the
real ArUco board textures with a true pinhole/homography projection at the
poses from the layout config, so detection difficulty scales realistically
with distance and viewing angle. The handout explicitly expects such an
instructor/team adapter. Perturbation parameters (brightness/blur/noise)
were added for the required robustness testing.

### Obstacles baked into map variants — not runtime-spawned objects
The sim's LiDAR raycasts against the occupancy grid, so obstacles must be
in the grid to be LiDAR-visible (a handout requirement). A bake script
rasterizes each obstacle set from the instructor YAML into its own map
(`track_clean`, `track_development`, `track_evaluation_A/B/C`), keeping
the instructor files untouched and every layout one launch argument away.

### Upstream compatibility patches — applied automatically
MuSHR's sim predates numpy 1.24 and crashes on removed aliases
(`np.float`/`np.int`) in three files. We patch them (one-word fixes,
version-controlled in `sim/patches/`) and the start script applies them
automatically every boot — after a manual patch step was skipped once and
cost a teammate an evening, the step was made impossible to skip.

## 2. Architecture decisions

### Single drive authority ("everyone publishes opinions, only the arbiter drives")
Every node publishes an opinion (steering suggestion or speed ceiling);
one arbiter computes v = min(curve, sign, lidar) and the steering blend,
and alone commands the car. Why: the min() rule is enforceable in exactly
one place; a crashed node degrades gracefully (its opinion goes stale
instead of fighting for the wheel); four people could build four nodes in
parallel against a fixed topic contract. This one decision shaped
everything else.

### Pure Pursuit — not PID steering, not Stanley
The vehicle kinematics are known, so we compute steering from geometry
(one line: the arc through a lookahead point) instead of regulating an
error signal with tuned gains. No integral state means the controller
recovers from any disturbance for free — teleport the car anywhere and it
re-acquires the line. A PID would add tuning burden to solve a problem
geometry already solves; speed-tracking PID exists in the VESC layer
beneath us, where it belongs.

### Predictive curvature speed planning — replacing a reactive prototype
First prototype set speed from the current bearing to the lookahead point
(reactive). It lapped, but measurably braked *in* corners rather than
before them. The replacement precomputes curvature from the waypoint
headings and caps each point at the minimum allowed speed over the next
3 m — braking moves before the corner. Decisions driven by measured data,
both times.

### Percentile clearances — never the raw minimum
The simulated LiDAR (like real hardware) produces single-ray phantom
short returns; our first obstacle-stop prototype false-triggered on one.
All safety clearances use the 10th percentile of an angular window: one
lying ray is ignored, a real obstacle spanning dozens of rays dominates
instantly.

### E-stop as the bottom of a linear ramp — not a separate mode
The speed cap ramps linearly from unconstrained (2.5 m clearance) to zero
(0.45 m). One formula covers gentle braking through full stop: no
threshold pair to disagree, no oscillation between modes. 0.45 m = car
length + laser mounting offset + one 50 ms control tick at top speed —
every safety number has a justification like this.

### Proportional, bounded dodge + urgency-weighted authority
The dodge steers toward the freer side, growing with proximity, capped
below Pure Pursuit's authority so it can never leave the track chasing
open space. That bound caused a real failure (evaluation B's on-line
obstacle → safe deadlock), fixed architecturally: the arbiter grants the
dodge full authority only as urgency approaches 1. Retested: passes with
zero recoveries needed.

### Recovery FSM inside the arbiter — not a separate node
Reversing means taking over the vehicle — a command decision, so it lives
in the one node with command authority (two drivers fighting the wheel is
how deadlocks become collisions). Stuck = no motion for 2 s while
commanding motion *or* pinned by the e-stop (the second clause was a bug
fix: a full e-stop sets commanded speed to 0, which made a naive
detector blind exactly when recovery mattered most).

### Hysteresis lap counting
A lap counts only when the nearest-waypoint index jumps from the last 10%
of the track into the first 10%; backwards crossings decrement. Noise or
wobbling on the line cannot fake a lap. Same philosophy as the percentile
clearances: state transitions require unambiguous evidence.

### Everything logged, always
The logger is part of the stack, not a debugging afterthought: every run
writes a 20 Hz CSV with every submission-required column, and one script
turns any CSV into the report figures. It caught a dead sim drivetrain on
its first outing; tuning decisions (curvature gain, dodge trigger) were
made by instrumenting which constraint was binding, not by feel.

## 3. Process decisions

- **Prototype first, formalize second:** every algorithm ran as a
  throwaway script (`testing/`) before becoming a node — the stack's
  architecture was informed by what the prototypes taught.
- **Verify by demonstration:** nothing was called done without a sim
  demonstration (lap counted, sign confirmed, recovery logged).
- **Guard against known failure modes structurally:** double-start
  refusal, automatic patching, node respawn, pre-run health check — every
  operational incident became a permanent guard, not a warning in a doc.
