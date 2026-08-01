# Presentation — slide-by-slide (5-minute talk + demo)

Build these in Google Slides / PowerPoint; every figure named here is in
`docs/figures/` (drag & drop). Speaker notes under each slide. Target:
~30 s per slide, leaving 1½ min for the video/live demo.

---

## Slide 1 — Title
**MuSHR Vision-Aware Autonomous Racing**
Team names · course · date. Background image: a Foxglove screenshot of
the car mid-lap (grab one from the video).

*Notes: one sentence — "We built a racing stack that laps autonomously,
reads its speed limits off trackside signs, dodges obstacles, and rescues
itself when stuck."*

## Slide 2 — The task
- 3 consecutive autonomous laps, no teleoperation
- Read ArUco speed signs (SLOW 1.0 / NORMAL 1.8 / BOOST 2.5 m/s) — zones latch until the next sign
- Avoid obstacles with LiDAR; recover from failures automatically
- Evaluated on **hidden layouts** → nothing can be hard-coded

*Notes: stress the hidden-layout point — it shaped every design choice.*

## Slide 3 — Architecture (the one rule)
Diagram: six nodes → arbiter → car (recreate the mermaid diagram from
ARCHITECTURE.md, or screenshot it from GitHub).
Headline: **"Everyone publishes opinions. Only the arbiter drives."**
- `v = min(v_curve, v_sign, v_lidar)` — the handout's formula, as one line of running code

*Notes: this is the slide to slow down on. Four people built four nodes
in parallel against this topic contract.*

## Slide 4 — Following & pacing the track
- Pure Pursuit: chase a point 1.2 m ahead; one line of circle geometry → steering. No PID — geometry over regulation (kinematics are known; the VESC layer beneath us does the PID job)
- Speed: curvature from waypoint headings, `v = v_max/(1+k|κ|)`, **minimum over the next 3 m** → brakes *before* corners
- Figure: `finalDev_tracking_error.png` (≤ 0.24 m all lap)

*Notes: mention we built a reactive speed law first, measured its late
braking, and replaced it — evidence-driven iteration.*

## Slide 5 — Perception: reading the rules
- ArUco DICT_4X4_50: binary-coded → near-zero false positives by construction
- Our filters: 3 consecutive frames to confirm; largest marker wins; latch until a different sign confirms
- Robustness table: 2/2 detections, 0 false transitions under −70 brightness and 7 px blur

*Notes: if asked about deep learning — "not needed; fiducial detection is
deterministic, and the assignment's difficulty is the filtering logic."*

## Slide 6 — Safety: the reflexes
- 720 rays → three robust clearances (10th percentile, never the min — the laser hallucinates single-ray phantoms; we caught one stopping the car)
- Speed cap = linear ramp: free at 2.5 m → **zero at 0.45 m**. E-stop is the ramp's bottom, not a separate mode
- Dodge grows with urgency toward the freer side; arbiter grants it full authority only near stopping distance
- Figure: `finalDev_clearance.png`

*Notes: every threshold has a measured justification — say "0.45 = car
length + sensor offset + one control tick at top speed" if asked.*

## Slide 7 — The war story (best 60 seconds of the talk)
**Evaluation B put a box dead on the racing line.**
1. First result: car stops safely… forever. Safe deadlock — dodge was
   *designed* weaker than Pure Pursuit, and PP aims through the box
2. Fix: urgency-weighted authority blending in the arbiter
3. Bonus bug found while testing the fix: recovery was unreachable during
   a full e-stop (stuck-detector required commanded speed > 0)
4. Retested: B passes with **zero recoveries needed**; recovery proven
   separately (teleport-into-box → reverse → resume, on video)

*Notes: this demonstrates systematic testing → root cause → architectural
fix → re-verification. It's worth more than any lap time.*

## Slide 8 — Results
The README results table (dev 46.5 s / A 63.2 / B 68.6 / C 80.9, all 0
collisions) + figure `finalA_speed_caps.png`.
Headline: **"Pace is legality":** eval A spends 60% of the lap in SLOW
zones — the car's speed is set by signs it reads, not by tuning.

*Notes: if asked why eval maps are slower — point at the green cap line
sitting at 1.0. The car is obeying, not struggling.*

## Slide 9 — Demo
Play the video(s): 3 laps (counter ticking to 3) → barrel dodge →
recovery. Or live demo via RUNBOOK if the setup allows (cold start ~2
min; health check first; teleport-into-box command ready in a terminal).

*Notes: if live, narrate from `rostopic echo /race/state` during the
recovery — RACING → REVERSING → RACING.*

## Slide 10 — What we'd do next
- Speed-dependent lookahead + racing-line optimization (biggest legal lap-time win)
- Localization robustness (currently ground-truth odom, switchable by parameter)
- Side/rear awareness (safety windows are forward-only)

*Notes: honest limitations land better than perfection claims. Then
questions — the ARCHITECTURE doc's Q&A section has the three likely ones
pre-answered.*

---

## Q&A cheat sheet (whole team reads this)

- **Why no PID?** Geometry over regulation for steering; the VESC layer
  does speed PID beneath us; we add explicit accel limits on top.
- **Why no neural nets?** Fiducials are deterministic and robust (see
  perturbation table); the challenge was the decision logic, not
  detection.
- **What if a sign is never visible?** Latched zone persists (NORMAL
  default before any sign). One dev-layout sign is 90° off-axis from the
  racing line — we proved it's a sightline issue, not a detection issue.
- **How were thresholds chosen?** Measured, not guessed — every number in
  `config/params.yaml` has a justification in the BUILDLOG (several came
  from instrumented probes of which constraint was binding).
- **What breaks it?** An obstacle field with no feasible gap (assignment
  guarantees one exists); localization drift (untested); side impacts
  (forward-only safety cones).
