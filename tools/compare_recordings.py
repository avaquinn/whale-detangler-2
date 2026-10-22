#!/usr/bin/env python3
"""
Compare fetched recordings / FRAM dumps side by side.

  python tools/compare_recordings.py fram_20261006_143445.csv fram_20261020_135706.csv ...
  python tools/compare_recordings.py fram_*.csv

For each file: duration, achieved acceleration rate, gaps, restarts (and why), reset breadcrumbs,
whether the strain-gauge bridge was powered, and battery voltage / state of charge / temperature.

Battery note: the firmware reads the fuel gauge once a minute (Timing::BATTERY_READ_PERIOD_MS) and
the gauge itself averages, so these values show the battery's state, not millisecond dips.
"""

import collections
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import viewer as v  # noqa: E402

RESET_BITS = [(1, "power-on"), (2, "reset-pin"), (4, "brown-out"), (8, "watchdog")]
STAGES = {1: "bridge on", 2: "writing FRAM", 3: "reading fuel gauge"}


def reset_name(cause):
    names = [n for bit, n in RESET_BITS if cause & bit]
    return "+".join(names) if names else f"0x{cause:02X}"


def summarize(path):
    samples, events, cycles, boots = v.load_file(path)
    # BOOT lines carry the reset cause (load_file keeps only boot/fw, so read them directly)
    causes = collections.Counter()
    for line in open(path, encoding="utf-8", errors="replace"):
        p = v.strip_host_time(line.strip()).split(",")
        if p[0] == "BOOT" and len(p) >= 4 and p[3].strip().isdigit():
            causes[reset_name(int(p[3]))] += 1

    rec = [s for s in samples if s.partial]
    full = [s for s in samples if not s.partial]
    out = {"file": os.path.basename(path)}
    accel = rec if rec else samples
    if len(accel) >= 2:
        span = accel[-1].t - accel[0].t
        gaps = [b.t - a.t for a, b in zip(accel, accel[1:])]
        out["duration"] = f"{span:.1f} s"
        out["accel rate"] = f"{(len(accel) - 1) / span:.2f} Hz" if span > 0 else "-"
        out["gaps > 100 ms"] = str(sum(1 for g in gaps if g > 0.1))
        out["longest gap"] = f"{max(gaps) * 1000:.0f} ms"
    data_boots = sorted({s.boot for s in samples})
    out["boots with data"] = str(len(data_boots))
    out["restarts logged"] = ", ".join(f"{n}x {c}" for c, n in causes.most_common()) or "none"
    crumbs = collections.Counter(STAGES.get(e.d0, str(e.d0)) for e in events if e.name == "RESET_DURING")
    out["reset during"] = ", ".join(f"{n}x {c}" for c, n in crumbs.most_common()) or "none"

    #FIRED and FAULT skip full samples (no bridge, no battery readings), which invalidates power tests
    states = [v.STATE_NAMES.get(e.d0, str(e.d0)) for e in events if e.name == "STATE_CHANGE"]
    states += [v.STATE_NAMES.get(s.state, "?") for s in full[:1]]
    out["device state"] = " -> ".join(dict.fromkeys(states)) or "unknown (no boot in file)"
    if any(st in ("FIRED", "FAULT") for st in states):
        out["device state"] += "  ! no full samples in this state"
    faults = collections.Counter(e.d0 for e in events if e.name == "SENSOR_FAULT")
    names = {1: "accelerometer", 2: "pressure", 4: "fuel gauge", 8: "FRAM"}
    out["sensor faults"] = ", ".join(f"{n}x {names.get(k, k)}" for k, n in faults.items()) or "none"
    dead = sum(1 for s in accel if s.ax == s.ay == s.az == -4)  #0xFFFF x 4 mg: accelerometer not answering
    out["accel dead samples"] = f"{dead} of {len(accel)}" if dead else "0"

    with_p = [s for s in full if s.flags is not None and s.flags & 2]
    clipped = [s for s in full if s.flags is not None and s.flags & 128]
    if not full:
        out["bridge"] = "no full samples"
    elif with_p or clipped:
        out["bridge"] = f"ON (p_raw {full[-1].p_raw}{', clipped' if clipped else ''})"
    else:
        out["bridge"] = "OFF (no pressure readings)"

    batt = [(s.t, s.mv, s.soc) for s in full if s.mv]
    distinct = []
    for t, mv, soc in batt:  # the gauge is read once a minute; keep changes only
        if not distinct or distinct[-1][1] != mv:
            distinct.append((t, mv, soc))
    if batt:
        mvs = [b[1] for b in batt]
        out["battery start/end"] = f"{batt[0][1]} / {batt[-1][1]} mV"
        out["battery min/max"] = f"{min(mvs)} / {max(mvs)} mV"
        out["charge (SOC)"] = f"{batt[0][2] / 100:.1f} -> {batt[-1][2] / 100:.1f} %"
        out["gauge readings"] = f"{len(distinct)} distinct"
    else:
        out["battery start/end"] = "no readings"
    temps = [s.temp for s in full if s.temp is not None]
    if temps:
        out["temp_raw"] = f"{min(temps)}..{max(temps)}"
    return out


def main():
    paths = sys.argv[1:]
    if not paths:
        print(__doc__)
        sys.exit(1)
    results = [summarize(p) for p in paths]
    rows = []
    for r in results:
        for k in r:
            if k not in rows:
                rows.append(k)
    width = max(len(k) for k in rows) + 2
    cols = [max(len(r.get(k, "-")) for k in rows) + 2 for r in results]
    for k in rows:
        print(k.ljust(width) + "".join(r.get(k, "-").ljust(c) for r, c in zip(results, cols)))


if __name__ == "__main__":
    main()
