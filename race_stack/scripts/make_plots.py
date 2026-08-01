#!/usr/bin/env python3
"""Step 7: turn a race_logger CSV into the report's required plots.

Usage (inside the container, matplotlib is preinstalled):
    python3 make_plots.py /assignment/logs/<run>.csv
Writes PNGs next to the CSV: speed profile, tracking error, obstacle
clearance, and the caps-vs-speed arbiter picture. Also prints lap times.
"""
import csv
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def col(rows, name, cast=float):
    return [cast(r[name]) for r in rows]


def main(path):
    rows = list(csv.DictReader(open(path)))
    base = Path(path).with_suffix("")
    t0 = float(rows[0]["t"])
    t = [float(r["t"]) - t0 for r in rows]

    # lap times from lap-counter increments
    laps = []
    for a, b in zip(rows[:-1], rows[1:]):
        if int(b["lap"]) > int(a["lap"]):
            laps.append(float(b["t"]) - t0)
    for i in range(1, len(laps)):
        print("lap %d: %.1f s" % (i + 1, laps[i] - laps[i - 1]))
    if laps:
        print("first lap: %.1f s (from logger start)" % laps[0])

    # 1: speed + caps over time (the arbiter picture)
    plt.figure(figsize=(11, 4))
    plt.plot(t, col(rows, "cmd_speed"), label="commanded speed", lw=1.5)
    plt.plot(t, col(rows, "cap_curve"), label="curve cap", alpha=0.6)
    plt.plot(t, col(rows, "cap_sign"), label="sign cap", alpha=0.6)
    lidar = [min(v, 3.0) for v in col(rows, "cap_lidar")]  # clip 99s
    plt.plot(t, lidar, label="lidar cap (clipped)", alpha=0.6)
    plt.ylim(0, 3.0)
    plt.xlabel("time (s)"); plt.ylabel("m/s")
    plt.legend(loc="lower right", ncol=4, fontsize=8)
    plt.title("Commanded speed vs the three caps: v = min(curve, sign, lidar)")
    plt.tight_layout()
    plt.savefig(str(base) + "_speed_caps.png", dpi=130)

    # 2: speed vs track position (speed profile deliverable)
    plt.figure(figsize=(11, 3.5))
    plt.scatter(col(rows, "progress_m"), col(rows, "cmd_speed"),
                s=2, alpha=0.5)
    plt.xlabel("track position (m)"); plt.ylabel("speed (m/s)")
    plt.title("Speed profile around the track (all laps overlaid)")
    plt.tight_layout()
    plt.savefig(str(base) + "_speed_profile.png", dpi=130)

    # 3: tracking error vs track position
    plt.figure(figsize=(11, 3.5))
    plt.scatter(col(rows, "progress_m"), col(rows, "lat_err"), s=2, alpha=0.5)
    plt.xlabel("track position (m)"); plt.ylabel("lateral error (m)")
    plt.title("Tracking error around the track")
    plt.tight_layout()
    plt.savefig(str(base) + "_tracking_error.png", dpi=130)

    # 4: min LiDAR range over time (obstacle clearance)
    plt.figure(figsize=(11, 3.5))
    plt.plot(t, col(rows, "min_scan"), lw=0.8)
    plt.xlabel("time (s)"); plt.ylabel("min scan range (m)")
    plt.title("Closest LiDAR return over time (obstacle clearance)")
    plt.tight_layout()
    plt.savefig(str(base) + "_clearance.png", dpi=130)

    print("plots written next to", path)


if __name__ == "__main__":
    main(sys.argv[1])
