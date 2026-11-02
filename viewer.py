#!/usr/bin/env python3
"""Bird's-eye view of LD2450 targets, with tracking to cope with jitter and dropouts.

Usage:
    python viewer.py                    # auto-detect the ESP32 (Linux, macOS, Windows)
    python viewer.py --port /dev/ttyACM0     # Linux
    python viewer.py --port COM5             # Windows
    python viewer.py --port /dev/cu.usbmodem1101   # macOS
    python viewer.py --demo             # fake noisy targets with dropouts, no hardware needed

Tuning (try these first if sideways motion still looks bad):
    --alpha 0.35     smoothing, lower = smoother but laggier (default 0.45)
    --hold 1.5       seconds to keep a lost target on screen as a hollow ring (default 1.0)
    --gate 1200      max mm a target may move between frames and still be the same one (default 900)
    --min-hits 3     frames needed before a new target is drawn, higher = fewer ghost dots (default 2)

Install:  pip install -r requirements.txt
Close the PlatformIO serial monitor first (only one program can open the port).
"""
import argparse
import math
import random
import time
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Wedge

MAX_RANGE_MM = 6000
HALF_FOV_DEG = 60
TRAIL_LEN = 30
MAX_TRACKS = 3
COAST_AFTER_S = 0.25   # no update for this long -> draw as hollow "lost" ring
PALETTE = ["tab:red", "tab:blue", "tab:green", "tab:orange", "tab:purple", "tab:brown"]


ESPRESSIF_VID = 0x303A  # USB vendor ID of the ESP32-S3's native USB


def find_port():
    """Pick the ESP32's serial port: prefer an Espressif USB device, else the only port."""
    from serial.tools import list_ports

    ports = list(list_ports.comports())
    esp = [p.device for p in ports if p.vid == ESPRESSIF_VID]
    if esp:
        return esp[0]
    if len(ports) == 1:
        return ports[0].device
    found = ", ".join(p.device for p in ports) or "none"
    raise SystemExit(f"Could not pick a port automatically (found: {found}). Use --port.")


class SerialSource:
    """Returns every new radar frame since the last poll, as a list of (x, y) detections."""

    def __init__(self, port):
        import serial
        self.ser = serial.Serial(port, 115200, timeout=0)
        self.buf = b""

    def poll(self):
        frames = []
        self.buf += self.ser.read(8192)
        *lines, self.buf = self.buf.split(b"\n")
        for line in lines:
            line = line.strip().decode(errors="ignore")
            if not line.startswith("D,"):
                continue
            parts = line.split(",")
            if len(parts) != 10:
                continue
            try:
                v = [int(p) for p in parts[1:]]
            except ValueError:
                continue
            dets = []
            for i in range(3):
                x, y = v[i * 3], v[i * 3 + 1]
                if not (x == 0 and y == 0):
                    dets.append((float(x), float(y)))
            frames.append(dets)
        return frames


class DemoSource:
    """Fake targets at ~10 frames/s with noise and random dropouts."""

    def __init__(self):
        self.last = 0.0

    def poll(self):
        now = time.time()
        if now - self.last < 0.1:
            return []
        self.last = now
        t = now
        dets = []
        # close, fast sideways walker (the hard case)
        if random.random() > 0.25:
            dets.append((1400 * math.sin(t * 0.9) + random.gauss(0, 50), 1200 + random.gauss(0, 50)))
        # farther target drifting around
        if random.random() > 0.10:
            dets.append((-1500 + 600 * math.sin(t * 0.5) + random.gauss(0, 40),
                         3500 + 700 * math.cos(t * 0.4) + random.gauss(0, 40)))
        random.shuffle(dets)  # slot order changes frame to frame, like the real sensor
        return [dets]


class Track:
    def __init__(self, tid, x, y, now):
        self.id = tid
        self.x, self.y = x, y
        self.hits = 1
        self.last_seen = now
        self.trail = deque([(x, y)], maxlen=TRAIL_LEN)


class Tracker:
    """Keeps target identity stable (the sensor reorders its 3 slots freely),
    smooths jitter, and bridges short dropouts."""

    def __init__(self, alpha, hold_s, gate_mm, min_hits):
        self.alpha, self.hold_s, self.gate, self.min_hits = alpha, hold_s, gate_mm, min_hits
        self.tracks = []
        self.next_id = 0

    def update(self, dets, now):
        pairs = sorted(
            (math.hypot(t.x - d[0], t.y - d[1]), ti, di)
            for ti, t in enumerate(self.tracks)
            for di, d in enumerate(dets)
        )
        used_t, used_d = set(), set()
        for dist, ti, di in pairs:
            if dist > self.gate:
                break
            if ti in used_t or di in used_d:
                continue
            t = self.tracks[ti]
            x, y = dets[di]
            t.x += self.alpha * (x - t.x)
            t.y += self.alpha * (y - t.y)
            t.hits += 1
            t.last_seen = now
            t.trail.append((t.x, t.y))
            used_t.add(ti)
            used_d.add(di)

        for di, (x, y) in enumerate(dets):
            if di not in used_d and len(self.tracks) < MAX_TRACKS:
                self.tracks.append(Track(self.next_id, x, y, now))
                self.next_id += 1

    def prune(self, now):
        keep = []
        for t in self.tracks:
            limit = self.hold_s if t.hits >= self.min_hits else 0.3  # unconfirmed ghosts die fast
            if now - t.last_seen <= limit:
                keep.append(t)
        self.tracks = keep

    def visible(self):
        return [t for t in self.tracks if t.hits >= self.min_hits]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("--demo", action="store_true")
    ap.add_argument("--alpha", type=float, default=0.45)
    ap.add_argument("--hold", type=float, default=1.0)
    ap.add_argument("--gate", type=float, default=900)
    ap.add_argument("--min-hits", type=int, default=2)
    args = ap.parse_args()

    source = DemoSource() if args.demo else SerialSource(args.port or find_port())
    tracker = Tracker(args.alpha, args.hold, args.gate, args.min_hits)

    fig, ax = plt.subplots(figsize=(8, 7))
    ax.set_aspect("equal")
    ax.set_xlim(-MAX_RANGE_MM * 0.9, MAX_RANGE_MM * 0.9)
    ax.set_ylim(-300, MAX_RANGE_MM + 300)
    ax.set_xlabel("x (mm), left / right")
    ax.set_ylabel("y (mm), distance ahead")
    ax.set_title("LD2450 bird's-eye view")
    ax.grid(True, alpha=0.3)

    ax.add_patch(Wedge((0, 0), MAX_RANGE_MM, 90 - HALF_FOV_DEG, 90 + HALF_FOV_DEG,
                       facecolor="tab:gray", alpha=0.08, edgecolor="gray"))
    for r in range(1000, MAX_RANGE_MM + 1, 1000):
        ax.add_patch(Wedge((0, 0), r, 90 - HALF_FOV_DEG, 90 + HALF_FOV_DEG,
                           fill=False, edgecolor="gray", alpha=0.3, lw=0.6))
        ax.text(0, r, f"{r // 1000} m", ha="center", va="bottom", fontsize=8, color="gray")
    ax.plot(0, 0, marker="^", color="black", markersize=10)

    trail_lines = [ax.plot([], [], "-", alpha=0.35)[0] for _ in range(MAX_TRACKS)]
    dots = [ax.plot([], [], "o", markersize=14, markeredgewidth=2)[0] for _ in range(MAX_TRACKS)]
    labels = [ax.text(0, 0, "", fontsize=9) for _ in range(MAX_TRACKS)]
    status = ax.text(0.01, 0.99, "", transform=ax.transAxes, va="top", fontsize=9, color="dimgray")
    frame_times = deque(maxlen=50)

    def update(_tick):
        now = time.time()
        for dets in source.poll():
            tracker.update(dets, now)
            frame_times.append(now)
        tracker.prune(now)

        vis = tracker.visible()
        for i in range(MAX_TRACKS):
            if i >= len(vis):
                dots[i].set_data([], [])
                trail_lines[i].set_data([], [])
                labels[i].set_text("")
                continue
            t = vis[i]
            col = PALETTE[t.id % len(PALETTE)]
            lost = (now - t.last_seen) > COAST_AFTER_S
            dots[i].set_data([t.x], [t.y])
            dots[i].set_color(col)
            dots[i].set_markerfacecolor("none" if lost else col)
            trail_lines[i].set_data([p[0] for p in t.trail], [p[1] for p in t.trail])
            trail_lines[i].set_color(col)
            labels[i].set_position((t.x + 150, t.y + 150))
            labels[i].set_text(f"#{t.id}  {math.hypot(t.x, t.y) / 1000:.2f} m" + ("  (lost)" if lost else ""))

        recent = [ft for ft in frame_times if now - ft <= 1.0]
        status.set_text(f"{len(recent)} frames/s")
        return dots + trail_lines + labels + [status]

    anim = FuncAnimation(fig, update, interval=40, blit=False, cache_frame_data=False)
    plt.show()


if __name__ == "__main__":
    main()
