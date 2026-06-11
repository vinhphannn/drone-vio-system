#!/usr/bin/env python3
"""
drone_core terminal dashboard
Reads /tmp/drone_core_stats.json written by the C++ pipeline every N seconds
and displays a live-updating status panel.

Usage:
    # Terminal 1: run the pipeline
    cd ~/ros2_ws/drone_core && ./scripts/run.sh

    # Terminal 2: open dashboard
    cd ~/ros2_ws/drone_core && python3 tools/dashboard.py
"""

import json
import math
import os
import sys
import time
from pathlib import Path

from rich.console import Console
from rich.layout import Layout
from rich.live import Live
from rich.panel import Panel
from rich.table import Table
from rich.text import Text
from rich import box

STATS_FILE = Path("/tmp/drone_core_stats.json")
REFRESH_HZ = 50  # Dashboard refresh rate (50 Hz for ultra-smooth updates)

console = Console()


# ── Helpers ───────────────────────────────────────────────────────────────────

def load_stats() -> dict | None:
    """Read and parse the JSON stats file. Returns None on error."""
    try:
        return json.loads(STATS_FILE.read_text())
    except (FileNotFoundError, json.JSONDecodeError):
        return None


def color_rate(hz: float, target: float, warn: float = 0.7) -> str:
    """Return a colored string for a measured rate vs target."""
    ratio = hz / target if target > 0 else 0
    if ratio >= 0.95:
        return f"[bold green]{hz:6.1f} Hz[/]"
    elif ratio >= warn:
        return f"[yellow]{hz:6.1f} Hz[/]"
    else:
        return f"[bold red]{hz:6.1f} Hz[/]"


def bar(value: float, maximum: float, width: int = 20) -> str:
    """Simple ASCII progress bar."""
    filled = int(round(value / maximum * width)) if maximum > 0 else 0
    filled = min(filled, width)
    empty  = width - filled
    return "[green]" + "█" * filled + "[dim]" + "░" * empty + "[/]"


def feature_quality(n: int, max_n: int = 200) -> str:
    ratio = n / max_n
    if ratio > 0.7:  return f"[bold green]{n} pts — EXCELLENT[/]"
    if ratio > 0.4:  return f"[yellow]{n} pts — GOOD[/]"
    if ratio > 0.2:  return f"[orange1]{n} pts — POOR[/]"
    return f"[bold red]{n} pts — CRITICAL[/]"


def fmt_angle(deg: float) -> str:
    """Format angle with direction indicator."""
    arrow = "↑" if abs(deg) < 5 else ("↗" if deg > 0 else "↙")
    return f"{deg:+7.2f}°  {arrow}"


# ── Panel builders ────────────────────────────────────────────────────────────

def build_header(s: dict) -> Panel:
    t = s.get("t_elapsed_s", 0)
    h, rem = divmod(int(t), 3600)
    m, sec = divmod(rem, 60)
    tracking = s.get("tracking_ok", False)
    status = "[bold green]● RUNNING[/]" if tracking else "[bold red]● TRACKING LOST[/]"
    text = Text.from_markup(
        f"  {status}    uptime: {h:02d}:{m:02d}:{sec:02d}"
        f"   poses: {s.get('pose_count', 0):,}"
    )
    return Panel(text, title="[bold cyan]drone_core VIO Pipeline[/]", box=box.DOUBLE)


def build_imu_panel(s: dict) -> Panel:
    hz   = s.get("imu_rate_hz",  0.0)
    rx   = s.get("imu_received", 0)
    drop = s.get("imu_dropped",  0)
    drop_pct = drop / rx * 100 if rx > 0 else 0.0

    t = Table(show_header=False, box=None, padding=(0, 1))
    t.add_column("k", style="bold", width=18)
    t.add_column("v")

    t.add_row("Rate",     color_rate(hz, 1600, warn=0.8))
    t.add_row("",         bar(hz, 1600))
    t.add_row("Received", f"[cyan]{rx:,}[/]")
    t.add_row("Dropped",  f"[{'red' if drop > 0 else 'green'}]{drop:,}  ({drop_pct:.2f}%)[/]")
    t.add_row("Device",   "[dim]/dev/ttyACM0 USB CDC[/]")
    t.add_row("Sensor",   "[dim]BMI160 ±4g / ±2000°/s[/]")

    return Panel(t, title="[bold yellow]⚡ IMU[/]", box=box.ROUNDED)


def build_camera_panel(s: dict) -> Panel:
    hz   = s.get("cam_rate_hz",  0.0)
    cap  = s.get("cam_captured", 0)
    drop = s.get("cam_dropped",  0)
    feats = s.get("feature_count", 0)

    t = Table(show_header=False, box=None, padding=(0, 1))
    t.add_column("k", style="bold", width=18)
    t.add_column("v")

    t.add_row("Rate",         color_rate(hz, 30.0))
    t.add_row("",             bar(hz, 30.0))
    t.add_row("Captured",     f"[cyan]{cap:,}[/]")
    t.add_row("Dropped",      f"[{'red' if drop > 0 else 'green'}]{drop:,}[/]")
    t.add_row("Image Quality", feature_quality(feats))
    t.add_row("",             bar(feats, 200))
    t.add_row("Resolution",   "[dim]640×480 mono8 MJPEG[/]")

    return Panel(t, title="[bold magenta]📷 Camera[/]", box=box.ROUNDED)


def build_vio_panel(s: dict) -> Panel:
    px    = s.get("px", 0.0);  py = s.get("py", 0.0);  pz = s.get("pz", 0.0)
    qw    = s.get("qw", 1.0);  qx = s.get("qx", 0.0)
    qy    = s.get("qy", 0.0);  qz = s.get("qz", 0.0)
    roll  = s.get("roll_deg",  0.0)
    pitch = s.get("pitch_deg", 0.0)
    yaw   = s.get("yaw_deg",   0.0)
    trk   = s.get("tracking_ok", False)

    dist = math.sqrt(px**2 + py**2 + pz**2)

    t = Table(show_header=False, box=None, padding=(0, 1))
    t.add_column("k", style="bold", width=18)
    t.add_column("v")

    trk_str = "[bold green]OK[/]" if trk else "[bold red]LOST[/]"
    t.add_row("Tracking",  trk_str)
    t.add_row("─" * 16,    "")
    t.add_row("X (forward)",  f"[cyan]{px:+8.3f} m[/]")
    t.add_row("Y (right)",    f"[cyan]{py:+8.3f} m[/]")
    t.add_row("Z (down)",     f"[cyan]{pz:+8.3f} m[/]")
    t.add_row("Distance",     f"[bold cyan]{dist:.3f} m[/]")
    t.add_row("─" * 16,    "")
    t.add_row("Roll",      fmt_angle(roll))
    t.add_row("Pitch",     fmt_angle(pitch))
    t.add_row("Yaw",       fmt_angle(yaw))
    t.add_row("─" * 16,    "")
    t.add_row("Quaternion", f"[dim]w={qw:.3f} x={qx:.3f} y={qy:.3f} z={qz:.3f}[/]")

    return Panel(t, title="[bold green]🛩  VIO Odometry[/]", box=box.ROUNDED)


def build_trajectory(s: dict, history: list) -> Panel:
    """Mini ASCII top-down XY trajectory plot."""
    W, H = 36, 12
    grid = [[" "] * W for _ in range(H)]

    # Draw center cross
    cx, cy = W // 2, H // 2
    for x in range(W): grid[cy][x] = "·"
    for y in range(H): grid[y][cx] = "·"
    grid[cy][cx] = "+"

    # Scale: auto-fit to ±2m or actual range
    xs = [p[0] for p in history] if history else [0.0]
    ys = [p[1] for p in history] if history else [0.0]
    scale = max(max(abs(x) for x in xs), max(abs(y) for y in ys), 0.5)

    def to_grid(x, y):
        gx = int(cx + x / scale * (W // 2 - 1))
        gy = int(cy - y / scale * (H // 2 - 1))
        return max(0, min(W-1, gx)), max(0, min(H-1, gy))

    # Draw trail
    for i, (hx, hy) in enumerate(history[-60:]):
        gx, gy = to_grid(hx, hy)
        grid[gy][gx] = "·" if i < len(history) - 6 else "○"

    # Draw current position
    px, py = s.get("px", 0.0), s.get("py", 0.0)
    gx, gy = to_grid(px, py)
    grid[gy][gx] = "[bold red]✦[/]"

    lines = ["".join(row) for row in grid]
    content = "\n".join(lines)
    content += f"\n  [dim]Scale: ±{scale:.2f}m  N={len(history)} pts[/]"

    return Panel(content, title="[bold blue]📍 XY Trajectory (top view)[/]", box=box.ROUNDED)


def build_waiting() -> Panel:
    return Panel(
        Text.from_markup(
            "\n  [yellow]Waiting for drone_core to start...[/]\n\n"
            "  Run in another terminal:\n"
            "  [bold]cd ~/ros2_ws/drone_core && ./scripts/run.sh[/]\n\n"
            f"  Watching: [dim]{STATS_FILE}[/]\n"
        ),
        title="[bold cyan]drone_core Dashboard[/]",
        box=box.DOUBLE
    )


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    history: list[tuple[float, float]] = []
    console.clear()

    with Live(console=console, refresh_per_second=REFRESH_HZ, screen=True) as live:
        while True:
            s = load_stats()

            if s is None:
                live.update(build_waiting())
                time.sleep(1.0 / REFRESH_HZ)
                continue

            # Track XY history for trajectory plot
            px, py = s.get("px", 0.0), s.get("py", 0.0)
            if not history or abs(px - history[-1][0]) > 0.005 or abs(py - history[-1][1]) > 0.005:
                history.append((px, py))
                history = history[-200:]  # Keep last 200 points

            # ── Layout ────────────────────────────────────────────────────────
            layout = Layout()
            layout.split_column(
                Layout(build_header(s),        name="header",     size=3),
                Layout(name="body"),
                Layout(name="footer",           size=3),
            )
            layout["body"].split_row(
                Layout(name="left",  ratio=1),
                Layout(name="right", ratio=1),
            )
            layout["left"].split_column(
                Layout(build_imu_panel(s),    name="imu",    ratio=1),
                Layout(build_camera_panel(s), name="camera", ratio=1),
            )
            layout["right"].split_column(
                Layout(build_vio_panel(s),         name="vio",  ratio=2),
                Layout(build_trajectory(s, history), name="traj", ratio=1),
            )

            # Footer: quick commands
            footer_text = Text.from_markup(
                "  [dim]Q=quit   Stats updated every 5s by pipeline   "
                "Pose log → /tmp/drone_poses.csv[/]"
            )
            layout["footer"].update(Panel(footer_text, box=box.SIMPLE))

            live.update(layout)
            time.sleep(1.0 / REFRESH_HZ)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        console.print("\n[yellow]Dashboard closed.[/]")
