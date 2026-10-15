#!/usr/bin/env python3
"""
Build a linear fit `dBm = m * cyd_bin + b` from a simultaneous CYD specstream
+ HackRF sweep capture. Writes:
  - scatter plot: HackRF dBm (y) vs averaged CYD bin level (x) per frequency
  - the fit line
  - residuals histogram
  - the two parameters so the firmware can later report real dBm

Usage:
    python calibrate_dbm.py --cyd cyd.log --hackrf hackrf.csv --out calib.png
"""
import argparse
import csv
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt


def parse_cyd(path):
    frames = []
    with open(path) as f:
        for line in f:
            s = line.strip()
            if not s.startswith("spec,"):
                continue
            p = s.split(",")
            try:
                ts = int(p[1]); ch = int(p[2]); span = int(p[3]); n = int(p[4])
                bins = [int(x) for x in p[5 : 5 + n]]
                if len(bins) == n:
                    frames.append((ts, ch, span, bins))
            except Exception:
                pass
    return frames


def cyd_bin_mhz(bin_idx, wifi_ch, span_mode, fft_n):
    bw = {0: 16, 1: 80, 2: 40}.get(span_mode, 16)
    center = 2407 + wifi_ch * 5
    return center - bw / 2 + (bin_idx * bw) / fft_n


def parse_hackrf(path):
    freqs_dbm = {}
    with open(path) as f:
        for row in csv.reader(f):
            if len(row) < 7:
                continue
            try:
                hz_lo = int(row[2]); bw = float(row[4])
                dbm = [float(x) for x in row[6:]]
                for i, d in enumerate(dbm):
                    fm = (hz_lo + i * bw) / 1e6
                    if 2400 <= fm <= 2485:
                        freqs_dbm.setdefault(fm, []).append(d)
            except Exception:
                pass
    freqs = sorted(freqs_dbm)
    avg = np.array([np.mean(freqs_dbm[f]) for f in freqs])
    return np.array(freqs), avg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cyd", required=True)
    ap.add_argument("--hackrf", required=True)
    ap.add_argument("--out", default="calib.png")
    args = ap.parse_args()

    cyd_frames = parse_cyd(args.cyd)
    if not cyd_frames:
        sys.exit("no CYD frames parsed")
    hrf_mhz, hrf_dbm = parse_hackrf(args.hackrf)
    if not hrf_mhz.size:
        sys.exit("no HackRF data parsed")

    # Average CYD frames (same tune assumed)
    _, ch, span, bins0 = cyd_frames[0]
    fft_n = len(bins0)
    sum_bins = np.zeros(fft_n, dtype=float)
    n_used = 0
    for ts, c, s, b in cyd_frames:
        if c == ch and s == span and len(b) == fft_n:
            sum_bins += np.array(b, dtype=float)
            n_used += 1
    cyd_avg = sum_bins / max(1, n_used)
    cyd_mhz = np.array([cyd_bin_mhz(i, ch, span, fft_n) for i in range(fft_n)])

    # Pair each CYD bin with the nearest HackRF 1 MHz bin
    pairs = []
    for i, fm in enumerate(cyd_mhz):
        idx = int(np.argmin(np.abs(hrf_mhz - fm)))
        if abs(hrf_mhz[idx] - fm) <= 1.5:
            pairs.append((cyd_avg[i], hrf_dbm[idx], fm))
    pairs = np.array(pairs)
    cyd_x = pairs[:, 0]
    hrf_y = pairs[:, 1]

    # Linear fit (least squares) + R^2
    m, b = np.polyfit(cyd_x, hrf_y, 1)
    pred = m * cyd_x + b
    ss_res = np.sum((hrf_y - pred) ** 2)
    ss_tot = np.sum((hrf_y - hrf_y.mean()) ** 2)
    r2 = 1 - ss_res / max(ss_tot, 1e-9)
    residuals = hrf_y - pred
    rmse = float(np.sqrt(np.mean(residuals ** 2)))

    print(f"paired  : {len(pairs)} CYD bins ↔ HackRF freqs")
    print(f"fit     : dBm = {m:.4f} * cyd_bin + {b:.2f}")
    print(f"R^2     : {r2:.3f}")
    print(f"RMSE    : {rmse:.2f} dB")
    print(f"range   : cyd {cyd_x.min():.1f}..{cyd_x.max():.1f}  "
          f"hrf {hrf_y.min():.1f}..{hrf_y.max():.1f} dBm")

    # Three-panel plot
    fig, axs = plt.subplots(1, 3, figsize=(16, 5))

    axs[0].scatter(cyd_x, hrf_y, s=20, alpha=0.6, color="tab:blue")
    xr = np.linspace(cyd_x.min(), cyd_x.max(), 50)
    axs[0].plot(xr, m * xr + b, color="tab:red",
                label=f"dBm = {m:.3f}*bin + {b:.2f}\nR²={r2:.3f}, RMSE={rmse:.2f} dB")
    axs[0].set_xlabel("CYD bin level (0..255)")
    axs[0].set_ylabel("HackRF dBm")
    axs[0].set_title("Scatter + linear fit")
    axs[0].grid(alpha=0.3)
    axs[0].legend(fontsize=9)

    axs[1].hist(residuals, bins=25, color="tab:orange", edgecolor="black")
    axs[1].axvline(0, color="tab:red", linestyle="--")
    axs[1].set_xlabel("Residual (dB)")
    axs[1].set_ylabel("Count")
    axs[1].set_title(f"Residuals, RMSE {rmse:.2f} dB")
    axs[1].grid(alpha=0.3)

    axs[2].plot(cyd_mhz, cyd_avg, color="tab:orange", label="CYD bin avg")
    axs[2].set_xlabel("Frequency (MHz)")
    axs[2].set_ylabel("CYD bin", color="tab:orange")
    ax2b = axs[2].twinx()
    ax2b.plot(hrf_mhz, hrf_dbm, color="tab:blue", label="HackRF dBm")
    ax2b.set_ylabel("HackRF dBm", color="tab:blue")
    axs[2].set_title("Aligned per freq")
    axs[2].grid(alpha=0.3)

    plt.tight_layout()
    out = Path(args.out).resolve()
    plt.savefig(out, dpi=100)
    print(f"saved {out}")


if __name__ == "__main__":
    main()
