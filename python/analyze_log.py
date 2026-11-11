#swing analyzer - turns a saved recording (.bin from swing_dashboard.py --save) into the results
#only reports what is in the log
#  python analyze_log.py run.bin                          rate, jitter, dropped samples, swings
#  python analyze_log.py rot.bin --rotations --target-deg 720   gyro scale from known-angle turns
#  python analyze_log.py swings.bin --redetect --start-dps 400   re-run swing detection with new values
#  python analyze_log.py run.bin --csv run.csv            export samples
from __future__ import annotations

import argparse
import statistics as stats

import swingproto as sp


def load(path: str):
    p = sp.Parser()
    with open(path, "rb") as f:
        pkts = p.feed(f.read())
    samples = [x for x in pkts if isinstance(x, sp.Sample)]
    swings = [x for x in pkts if isinstance(x, sp.Swing)]
    status = [x for x in pkts if isinstance(x, sp.Status)]
    return samples, swings, status, p


def pct(sorted_vals, q):
    i = min(len(sorted_vals) - 1, max(0, round(q / 100 * (len(sorted_vals) - 1))))
    return sorted_vals[i]


def report_basic(samples, swings, status, parser, nominal_us: float):
    st = status[-1] if status else None
    print("=" * 64)
    if not samples:
        print("No samples in log.")
        if st:
            print(f"IMU status {st.bmi_status}: {sp.BMI_STATUS_TEXT.get(st.bmi_status, '?')}")
        return
    if st is None:
        print("WARNING: no status packet in log; assuming 180 MHz clock. Record > 1 s.")
    core_hz = st.core_hz if st else 180_000_000
    fs = st.gyro_fs_dps if st else 2000

    #dropped samples from gaps in the sequence counter
    gaps, resets = 0, 0
    for a, b in zip(samples, samples[1:]):
        g = sp.seq_gap(a.seq, b.seq)
        if g is None:
            resets += 1
        else:
            gaps += g
    expected = len(samples) + gaps
    print("DROPPED SAMPLES (sequence counter)")
    if resets:
        print(f"  NOTE: board was reset {resets} time(s) during this log. For clean numbers,")
        print("        record a new log without pressing reset.")
    print(f"  expected {expected} samples (~{expected / 1000:.1f} s at 1 kHz), "
          f"received {len(samples)}, missing {gaps}")
    print(f"  loss rate: {gaps / expected * 100:.4f} %")
    if st:
        print(f"  board counters at last status: missed_reads={st.missed_reads} "
              f"ring_overflows={st.ring_overflows} tx_dropped={st.tx_dropped} spi_errors={st.spi_errors}")
    print(f"  PC side: bad CRC packets={parser.bad_crc}, skipped bytes={parser.skipped_bytes}")

    #sampling rate and jitter from the cycle counter timestamps
    periods_us = [((b.t_cyc - a.t_cyc) & 0xFFFFFFFF) / core_hz * 1e6
                  for a, b in zip(samples, samples[1:]) if b.seq == a.seq + 1]
    if periods_us:
        s = sorted(periods_us)
        mean = stats.fmean(periods_us)
        dev = [abs(p - nominal_us) for p in periods_us]
        print("\nSAMPLING RATE + JITTER (DWT cycle counter, measured by the MCU's own clock)")
        print(f"  intervals analysed: {len(periods_us)} (gaps excluded), core clock {core_hz} Hz")
        print(f"  mean period {mean:.3f} us  ->  rate {1e6 / mean:.3f} Hz")
        print(f"  period min {s[0]:.3f} us, max {s[-1]:.3f} us, peak-to-peak {s[-1] - s[0]:.3f} us")
        print(f"  std dev {stats.pstdev(periods_us):.3f} us")
        print(f"  max |period - {nominal_us:g} us| = {max(dev):.3f} us")
        print(f"  99.9th percentile period {pct(s, 99.9):.3f} us")
        print("  NOTE: rate accuracy here is only as good as the MCU clock. Confirm the")
        print("        rate with the scope on pin D7.")
    if st:
        print(f"  (configured timer rate: {st.rate_mhz / 1000:.3f} Hz - setting, not a measurement)")

    #swings found on the board
    print(f"\nSWINGS (detected on the board): {len(swings)}")
    for w in swings:
        clip = "  CLIPPED: true peak is higher than shown" if w.flags & sp.FLAG_GYRO_CLIPPED else ""
        print(f"  #{w.swing_id:3d}  peak {w.peak_dps(fs):7.1f} deg/s  "
              f"duration {(w.end_seq - w.start_seq + 1)} ms{clip}")


def report_rotations(samples, status, axis: str, target_deg: float, thresh_dps: float, margin_s: float):
    #gyro scale check without a turntable: turn the board by a known angle (one full turn against a
    #straight edge), stop, repeat. adds up the gyro over each turn and compares it with target_deg.
    #the speed of each turn doesn't matter, only the start and end positions.
    st = status[-1] if status else None
    fs = st.gyro_fs_dps if st else 2000
    k = "xyz".index(axis)
    if len(samples) < 3000:
        print("Log too short for the rotation check.")
        return
    vals = [s.gyr[k] * fs / 32768 for s in samples]
    seqs = [s.seq for s in samples]
    margin = int(margin_s * 1000)

    def find_turns(bias):
        #stretches where the board is clearly turning, grown by the margin on each side
        turns, start = [], None
        for i, v in enumerate(vals + [bias]):
            if abs(v - bias) > thresh_dps:
                start = i if start is None else start
            elif start is not None:
                a, b = max(0, start - margin), min(len(vals), i + margin)
                if turns and a <= turns[-1][1]:
                    turns[-1] = (turns[-1][0], b)    #merge turns that touch
                else:
                    turns.append((a, b))
                start = None
        return turns

    #zero-rate bias from everything outside the turns (first guess from the first second)
    bias = stats.fmean(vals[:1000])
    turns = find_turns(bias)
    moving = set()
    for a, b in turns:
        moving.update(range(a, b))
    still = [v for i, v in enumerate(vals) if i not in moving]
    if len(still) > 1000:
        bias = stats.fmean(still)
        turns = find_turns(bias)

    print("=" * 64)
    print(f"ROTATION CHECK on gyro {axis.upper()} axis, target {target_deg:g} deg per turn")
    print(f"  zero-rate bias from {len(still)} still samples: {bias:+.3f} deg/s (subtracted)")
    if not turns:
        print("  no turns found. Check --axis and that the board was turned.")
        return
    errs = []
    for n, (a, b) in enumerate(turns, 1):
        gap = seqs[b - 1] - seqs[a] + 1 - (b - a)
        angle = sum(v - bias for v in vals[a:b]) / 1000.0   #1 ms per sample
        clip = any(samples[i].flags & sp.FLAG_GYRO_CLIPPED for i in range(a, b))
        err = (abs(angle) - target_deg) / target_deg * 100
        errs.append(err)
        note = ("  CLIPPED" if clip else "") + (f"  {gap} samples missing" if gap else "")
        print(f"  turn {n}: {angle:+8.2f} deg   error {err:+.2f} %   ({(b - a) / 1000:.1f} s){note}")
    print(f"  turns: {len(errs)}   mean error {stats.fmean(errs):+.2f} %   "
          f"worst {max(errs, key=abs):+.2f} %")
    print("  the result also includes how exactly each turn was lined up by hand")


def main():
    ap = argparse.ArgumentParser(description="turns a saved swing analyzer recording into results")
    ap.add_argument("log")
    ap.add_argument("--nominal-us", type=float, default=1000.0, help="intended sample period")
    ap.add_argument("--axis", default="z", choices="xyz")
    ap.add_argument("--redetect", action="store_true", help="re-run swing detection on the PC")
    ap.add_argument("--start-dps", type=float, default=300)
    ap.add_argument("--end-dps", type=float, default=150)
    ap.add_argument("--csv", help="export samples to CSV")
    ap.add_argument("--rotations", action="store_true", help="known-angle turns instead of a turntable")
    ap.add_argument("--target-deg", type=float, default=360.0, help="angle of each turn")
    ap.add_argument("--rot-thresh", type=float, default=15.0, help="deg/s that counts as turning")
    ap.add_argument("--rot-margin", type=float, default=0.3, help="s added before/after each turn")
    a = ap.parse_args()

    samples, swings, status, parser = load(a.log)
    report_basic(samples, swings, status, parser, a.nominal_us)
    if a.rotations:
        report_rotations(samples, status, a.axis, a.target_deg, a.rot_thresh, a.rot_margin)
    if a.redetect:
        det = sp.SwingDetector(sp.SwingCfg(start_dps=a.start_dps, end_dps=a.end_dps))
        found = [e for s in samples if (e := det.update(s.seq, s.gyr, s.flags))]
        print(f"\nRE-DETECTION with start={a.start_dps:g} end={a.end_dps:g} deg/s: {len(found)} swings")
        for w in found:
            print(f"  #{w.swing_id:3d}  peak {w.peak_dps():7.1f} deg/s")
    if a.csv:
        fs = status[-1].gyro_fs_dps if status else 2000
        afs = status[-1].acc_fs_g if status else 16
        with open(a.csv, "w") as f:
            f.write("seq,t_cyc,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,flags\n")
            for s in samples:
                acc = ",".join(f"{v * afs / 32768:.4f}" for v in s.acc)
                gyr = ",".join(f"{v * fs / 32768:.3f}" for v in s.gyr)
                f.write(f"{s.seq},{s.t_cyc},{acc},{gyr},{s.flags}\n")
        print(f"\nWrote {len(samples)} rows to {a.csv}")


if __name__ == "__main__":
    main()
