#!/usr/bin/env python3
"""Per-round resource and throughput monitor (started by scripts/run_wuhu_round.sh).

Samples every --interval seconds until the mapper exits and appends one CSV row to <round>/resources.csv:

  wall, elapsed_s           wall clock (epoch s) and seconds since the monitor started
  frames, data_stamp        frames processed by the mapper so far (rows of the backend diagnostics.csv)
                            and the data timestamp of the newest one
  fps, rt_factor            processed frames per wall second and data seconds per wall second over the
                            last interval (rt_factor < 1: the mapper is slower than the recording)
  backend_ms                mean backend (RTK fusion) processing time of the frames of the interval
  mapper_rss_mb/cpu_pct     resident memory and CPU (% of one core) of fastlivo_mapping
  decoder_rss_mb/cpu_pct    camera decoder (image_transport republish, LIVO only)
  player_cpu_pct            raw-recording player
  sys_used_mb, swap_used_mb host memory in use (MemTotal - MemAvailable) and swap in use

    /usr/bin/python3 tools/livo/resmon.py <round_dir> --pid <mapper_pid> [--interval 2]
"""
import argparse
import os
import pathlib
import time

PATTERNS = {"mapper": ("fastlivo_mapping",), "decoder": ("image_transport", "republish"), "player": ("livo/player.py", "livo.player")}
HEADER = ("wall,elapsed_s,frames,data_stamp,fps,rt_factor,backend_ms,mapper_rss_mb,mapper_cpu_pct,"
          "decoder_rss_mb,decoder_cpu_pct,player_cpu_pct,sys_used_mb,swap_used_mb")
CLK = os.sysconf("SC_CLK_TCK")


def proc_table():
    """{pid: (cmdline, rss_kb, cpu_ticks)} of every readable process."""
    out = {}
    for d in os.listdir("/proc"):
        if not d.isdigit():
            continue
        try:
            cmd = open(f"/proc/{d}/cmdline", "rb").read().replace(b"\0", b" ").decode("utf-8", "replace")
            stat = open(f"/proc/{d}/stat").read()
            fields = stat[stat.rindex(")") + 2:].split()
            ticks = int(fields[11]) + int(fields[12])
            rss = 0
            for line in open(f"/proc/{d}/status"):
                if line.startswith("VmRSS:"):
                    rss = int(line.split()[1])
                    break
            out[int(d)] = (cmd, rss, ticks)
        except (OSError, ValueError, IndexError):
            continue
    return out


def meminfo():
    vals = {}
    try:
        for line in open("/proc/meminfo"):
            k, v = line.split(":", 1)
            vals[k] = int(v.split()[0])
    except OSError:
        return float("nan"), float("nan")
    used = (vals.get("MemTotal", 0) - vals.get("MemAvailable", 0)) / 1024
    swap = (vals.get("SwapTotal", 0) - vals.get("SwapFree", 0)) / 1024
    return used, swap


class DiagTail:
    """Incremental reader of the newest Log/fusion/*/diagnostics.csv (frames, last stamp, processing_ms)."""

    def __init__(self, round_dir):
        self.round_dir = pathlib.Path(round_dir)
        self.path, self.pos, self.cols, self.frames, self.stamp, self.partial = None, 0, None, 0, float("nan"), b""

    def poll(self):
        if self.path is None:
            runs = sorted((self.round_dir / "Log" / "fusion").glob("*/diagnostics.csv"))
            if not runs:
                return []
            self.path = runs[-1]
        try:
            with open(self.path, "rb") as f:
                f.seek(self.pos)
                data = f.read()
        except OSError:
            return []
        self.pos += len(data)
        data = self.partial + data
        lines = data.split(b"\n")
        self.partial = lines.pop()
        new_ms = []
        for raw in lines:
            line = raw.decode("utf-8", "replace").strip()
            if not line:
                continue
            parts = line.split(",")
            if self.cols is None:
                self.cols = {n: i for i, n in enumerate(parts)}
                continue
            try:
                self.stamp = float(parts[self.cols["stamp"]])
                new_ms.append(float(parts[self.cols["processing_ms"]]))
                self.frames += 1
            except (KeyError, ValueError, IndexError):
                continue
        return new_ms


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("round_dir")
    ap.add_argument("--pid", type=int, required=True, help="mapper pid; the monitor stops when it exits")
    ap.add_argument("--interval", type=float, default=2.0)
    a = ap.parse_args()
    out = pathlib.Path(a.round_dir) / "resources.csv"
    new = not out.exists()
    diag = DiagTail(a.round_dir)
    t0 = time.time()
    prev = None  # (wall, {pid: ticks}, frames, stamp)
    with open(out, "a", buffering=1) as f:
        if new:
            f.write(HEADER + "\n")
        while True:
            alive = os.path.exists(f"/proc/{a.pid}")
            wall = time.time()
            table = proc_table()
            ms = diag.poll()
            groups = {k: [pid for pid, (cmd, _, _) in table.items() if any(p in cmd for p in pats) and "resmon" not in cmd]
                      for k, pats in PATTERNS.items()}
            ticks = {pid: v[2] for pid, v in table.items()}

            def rss(k):
                return sum(table[p][1] for p in groups[k]) / 1024

            def cpu(k):
                if prev is None:
                    return float("nan")
                dt = wall - prev[0]
                used = sum(ticks[p] - prev[1].get(p, ticks[p]) for p in groups[k]) / CLK
                return 100.0 * used / dt if dt > 0 else float("nan")

            if prev is not None and wall > prev[0]:
                dt = wall - prev[0]
                fps = (diag.frames - prev[2]) / dt
                rt = (diag.stamp - prev[3]) / dt if prev[3] == prev[3] and diag.stamp == diag.stamp else float("nan")
            else:
                fps = rt = float("nan")
            used, swap = meminfo()
            bms = sum(ms) / len(ms) if ms else float("nan")
            f.write(f"{wall:.1f},{wall - t0:.1f},{diag.frames},{diag.stamp:.3f},{fps:.2f},{rt:.3f},{bms:.1f},"
                    f"{rss('mapper'):.0f},{cpu('mapper'):.0f},{rss('decoder'):.0f},{cpu('decoder'):.0f},"
                    f"{cpu('player'):.0f},{used:.0f},{swap:.0f}\n")
            prev = (wall, ticks, diag.frames, diag.stamp)
            if not alive:
                break
            time.sleep(a.interval)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
