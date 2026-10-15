#!/usr/bin/env python3
"""
Overlay a CYD `specdump` / `specstream` capture against a HackRF
`hackrf_sweep` CSV so we can visually confirm the CYD sees what the
HackRF sees in the 2.4 GHz ISM band.

Typical workflow:

    # 1) On the HackRF host, start a sweep over the ISM band
    hackrf_sweep -f 2400:2485 -w 1000000 -r hackrf.csv

    # 2) On the CYD serial, capture a stream of frames
    #    (do this at the same moment as the hackrf_sweep is running)
    #    Example from PowerShell:
    #      $p = New-Object System.IO.Ports.SerialPort "COM3",115200
    #      $p.Open(); $p.WriteLine("specstream 20")
    #      1..200 | % { $p.ReadLine() | Add-Content cyd.log }
    #      $p.Close()

    # 3) Overlay
    python compare_spectrum.py --cyd cyd.log --hackrf hackrf.csv

Produces compare.png: both spectra, normalized 0..1, over frequency MHz.
Mismatches between the two curves point to where the CYD lies about reality.
"""

import argparse
import csv
import sys
from datetime import datetime
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt


# ---------------------------------------------------------------------------
# CYD specdump parser
# ---------------------------------------------------------------------------
def parse_cyd(path):
    """
    Returns a list of (ts_ms, wifiCh, spanMode, bins[]) tuples parsed from
    one or more `spec,...` lines in `path`. Any non-matching lines are
    ignored silently so you can feed a full serial log.
    """
    frames = []
    with open(path, "r") as f:
        for line in f:
            s = line.strip()
            if not s.startswith("spec,"):
                continue
            parts = s.split(",")
            if len(parts) < 6:
                continue
            try:
                ts     = int(parts[1])
                ch     = int(parts[2])
                span   = int(parts[3])
                n      = int(parts[4])
                bins   = [int(x) for x in parts[5 : 5 + n]]
                if len(bins) != n:
                    continue
                frames.append((ts, ch, span, bins))
            except (ValueError, IndexError):
                continue
    return frames


def cyd_bin_mhz(bin_idx, wifi_ch, span_mode, fft_n):
    """
    Reproduces hopBinToBtCh's bin->freq math.
    span_mode values match SpanMode enum: 0=S16, 1=S80, 2=S40.
    """
    bw_mhz = {0: 16, 1: 80, 2: 40}.get(span_mode, 16)
    center = 2407 + wifi_ch * 5          # same formula as firmware
    return center - bw_mhz / 2 + (bin_idx * bw_mhz) / fft_n


# ---------------------------------------------------------------------------
# HackRF hackrf_sweep CSV parser
# ---------------------------------------------------------------------------
def parse_hackrf(path):
    """
    hackrf_sweep CSV lines look like:
      date, time.us, hz_low, hz_high, bin_width_hz, num_samples, dBm0, dBm1, ...

    Returns a list of (datetime, {freq_mhz: dbm}) tuples. One sweep can span
    multiple CSV rows (hackrf_sweep breaks the full range into chunks); we
    merge rows that share the same timestamp into one sweep.
    """
    sweeps = {}
    with open(path, "r", newline="") as f:
        reader = csv.reader(f)
        for row in reader:
            if len(row) < 7:
                continue
            try:
                date_s = row[0].strip()
                time_s = row[1].strip()
                dt = datetime.strptime(f"{date_s} {time_s}",
                                       "%Y-%m-%d %H:%M:%S.%f")
                hz_lo = int(row[2])
                bin_w = float(row[4])
                dbm   = [float(x) for x in row[6:]]
                buf   = sweeps.setdefault(dt, {})
                for i, d in enumerate(dbm):
                    f_mhz = (hz_lo + i * bin_w) / 1e6
                    # Clip to ISM band for a clean overlay
                    if 2400 <= f_mhz <= 2485:
                        buf[f_mhz] = d
            except (ValueError, IndexError):
                continue
    return list(sweeps.items())


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def normalize(values):
    v = np.asarray(values, dtype=float)
    span = v.max() - v.min()
    if span < 1e-9:
        return np.zeros_like(v)
    return (v - v.min()) / span


def cyd_average_frame(frames):
    """Average all CYD frames (assumes the same tune state). Returns (mhz[], levels[])."""
    if not frames:
        return np.array([]), np.array([])
    _, ch, span, bins0 = frames[0]
    fft_n  = len(bins0)
    sum_bins = np.zeros(fft_n, dtype=float)
    for ts, c, s, bins in frames:
        if c != ch or s != span or len(bins) != fft_n:
            continue    # skip frames with mismatched tune (user retuned mid-capture)
        sum_bins += np.array(bins, dtype=float)
    sum_bins /= max(1, len(frames))
    mhz = np.array([cyd_bin_mhz(i, ch, span, fft_n) for i in range(fft_n)])
    return mhz, sum_bins


def hackrf_average_sweeps(sweeps):
    """Average dBm over all sweeps per frequency. Returns (freq_mhz[], dbm[])."""
    freq_accum = {}
    for _, d in sweeps:
        for f, dbm in d.items():
            freq_accum.setdefault(f, []).append(dbm)
    freqs = sorted(freq_accum.keys())
    dbm_avg = np.array([np.mean(freq_accum[f]) for f in freqs])
    return np.array(freqs), dbm_avg


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(
        description="Overlay CYD specdump against HackRF sweep")
    ap.add_argument("--cyd",    required=True, help="CYD specstream log")
    ap.add_argument("--hackrf", required=True, help="hackrf_sweep CSV")
    ap.add_argument("--out",    default="compare.png", help="Output PNG")
    ap.add_argument("--raw",    action="store_true",
                    help="Plot absolute units (CYD 0..255, HackRF dBm) "
                         "on two y-axes instead of normalized overlay")
    ap.add_argument("--linear", action="store_true",
                    help="Convert HackRF dBm to linear mW before normalizing "
                         "so the shape comparison with CYD is apples-to-apples "
                         "(CYD bin values are already linear).")
    args = ap.parse_args()

    cyd_frames = parse_cyd(args.cyd)
    if not cyd_frames:
        sys.exit(f"No `spec,...` lines parsed from {args.cyd}")
    hackrf_sweeps = parse_hackrf(args.hackrf)
    if not hackrf_sweeps:
        sys.exit(f"No HackRF sweeps parsed from {args.hackrf}")

    cyd_mhz, cyd_lvl    = cyd_average_frame(cyd_frames)
    hrf_mhz, hrf_dbm    = hackrf_average_sweeps(hackrf_sweeps)

    print(f"CYD    : {len(cyd_frames):4d} frames, ch={cyd_frames[0][1]} "
          f"span={cyd_frames[0][2]}, {len(cyd_lvl)} bins, "
          f"range {cyd_mhz[0]:.1f}..{cyd_mhz[-1]:.1f} MHz")
    print(f"HackRF : {len(hackrf_sweeps):4d} sweeps, "
          f"{len(hrf_dbm)} freq bins, "
          f"range {hrf_mhz[0]:.1f}..{hrf_mhz[-1]:.1f} MHz")

    fig, ax1 = plt.subplots(figsize=(12, 6))
    if args.raw:
        ax1.plot(hrf_mhz, hrf_dbm, color="tab:blue",
                 label="HackRF (dBm)")
        ax1.set_ylabel("HackRF dBm", color="tab:blue")
        ax1.tick_params(axis="y", labelcolor="tab:blue")
        ax2 = ax1.twinx()
        ax2.plot(cyd_mhz, cyd_lvl, color="tab:orange",
                 label=f"CYD (0..255, ch{cyd_frames[0][1]} span{cyd_frames[0][2]})")
        ax2.set_ylabel("CYD bin level", color="tab:orange")
        ax2.tick_params(axis="y", labelcolor="tab:orange")
    else:
        if args.linear:
            # dBm → mW (linear power) so both scales represent power
            hrf_vals = 10.0 ** (hrf_dbm / 10.0)
            hrf_label = "HackRF (linear mW, normalized)"
        else:
            hrf_vals = hrf_dbm
            hrf_label = "HackRF dBm (normalized)"
        ax1.plot(hrf_mhz, normalize(hrf_vals), color="tab:blue",
                 label=hrf_label)
        ax1.plot(cyd_mhz, normalize(cyd_lvl), color="tab:orange",
                 label=f"CYD ch{cyd_frames[0][1]} span{cyd_frames[0][2]} (linear, normalized)")
        ax1.set_ylabel("Normalized power")
        ax1.legend(loc="upper right")

    # Mark BLE adv channels for reference
    for mhz, lbl in [(2402, "adv 37"), (2426, "adv 38"), (2480, "adv 39")]:
        ax1.axvline(mhz, color="tab:red", alpha=0.3, linestyle="--")
        ax1.text(mhz, ax1.get_ylim()[1] * 0.98, lbl,
                 fontsize=8, color="tab:red", ha="center", va="top")
    # Mark WiFi channel centers 1/6/11
    for ch, mhz in [(1, 2412), (6, 2437), (11, 2462)]:
        ax1.axvline(mhz, color="tab:green", alpha=0.3, linestyle=":")
        ax1.text(mhz, ax1.get_ylim()[1] * 0.90, f"wifi {ch}",
                 fontsize=8, color="tab:green", ha="center", va="top")

    ax1.set_xlabel("Frequency (MHz)")
    ax1.set_xlim(2399, 2486)
    ax1.set_title("CYD vs HackRF spectrum overlay (2.4 GHz ISM band)")
    ax1.grid(True, alpha=0.3)
    plt.tight_layout()

    out_path = Path(args.out).resolve()
    plt.savefig(out_path, dpi=100)
    print(f"Saved {out_path}")


if __name__ == "__main__":
    main()
