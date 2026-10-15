#!/usr/bin/env python3
"""Analyze a timed TX-sweep capture from hackrf_sweep (narrow 2435-2450 MHz)."""
import csv
import sys
from datetime import datetime
import numpy as np


def main(path):
    rows = []    # (dt, freq_mhz, dbm)
    with open(path) as f:
        for r in csv.reader(f):
            if len(r) < 7:
                continue
            try:
                dt = datetime.strptime(f"{r[0].strip()} {r[1].strip()}",
                                       "%Y-%m-%d %H:%M:%S.%f")
                hz_lo = int(r[2]); bw = float(r[4])
                dbm = [float(x) for x in r[6:]]
                for i, d in enumerate(dbm):
                    fm = (hz_lo + i*bw) / 1e6
                    if 2440 <= fm <= 2444 or 2436 <= fm <= 2437 or 2448 <= fm <= 2449:
                        rows.append((dt, fm, d))
            except Exception:
                pass
    if not rows:
        sys.exit("no data")

    t0 = min(r[0] for r in rows)
    rel = [((r[0]-t0).total_seconds(), r[1], r[2]) for r in rows]

    # Timing offsets of the TX phases relative to hackrf_sweep start.
    # Script delayed 3s after starting sweep before opening serial, then 2s
    # before first TX. Each phase: 5s TX + 2s stop = 7s.
    phases = [
        ("baseline",   0.5,  3.5),
        ("bo=0  (max)",   6.0, 8.5),
        ("bo=40 (-10dB)", 13.0, 15.5),
        ("bo=80 (-20dB)", 20.0, 22.5),
        ("bo=120(-30dB)", 27.0, 29.5),
        ("bo=160(-40dB)", 34.0, 36.5),
        ("bo=200(-50dB)", 41.0, 43.5),
    ]

    print("{0:<18} {1:>9} {2:>9} {3:>9}".format(
        "phase", "peak_dBm", "peak_MHz", "floor_dBm"))
    results = []
    for name, lo, hi in phases:
        near_bins = [(f, d) for t, f, d in rel if lo <= t <= hi and 2440 <= f <= 2444]
        far_bins  = [(f, d) for t, f, d in rel if lo <= t <= hi and (f <= 2437 or f >= 2448)]
        if not near_bins:
            print(f"{name:<18}  NO SAMPLES")
            continue
        peak = max(near_bins, key=lambda x: x[1])
        floor = np.mean([d for _, d in far_bins]) if far_bins else float("nan")
        print(f"{name:<18} {peak[1]:9.1f} {peak[0]:9.2f} {floor:9.1f}")
        results.append((name, peak[1]))

    # Compute deltas vs bo=0 (if present)
    if len(results) >= 2:
        bo0 = None
        for n, p in results:
            if "bo=0" in n:
                bo0 = p
                break
        if bo0 is not None:
            print()
            print("Observed delta vs bo=0 (CYD max):")
            exp_map = {"bo=0": 0, "bo=40": -10, "bo=80": -20,
                       "bo=120": -30, "bo=160": -40, "bo=200": -50}
            print("{0:<18} {1:>9} {2:>9} {3:>9}".format(
                "phase", "obs_dB", "exp_dB", "resid_dB"))
            for name, p in results:
                for key, exp in exp_map.items():
                    if key in name:
                        obs = p - bo0
                        resid = obs - exp
                        print(f"{name:<18} {obs:+9.1f} {exp:+9d} {resid:+9.1f}")
                        break


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1
         else r"C:\Users\bkenn\ESP32\signalhound-cyd\scripts\tx.csv")
