"""Find the top-level X11 window of a process (used to embed RViz).

Reads the window manager's client list and each window's _NET_WM_PID with
xprop, which ships with every Ubuntu desktop (x11-utils). X11 only: on a
Wayland session no window is found and the 3D view stays empty.
"""
from __future__ import annotations

import os
import re
import subprocess


def _xprop(*args: str) -> str:
    try:
        result = subprocess.run(["xprop", *args], capture_output=True, text=True, timeout=3.0)
        return result.stdout if result.returncode == 0 else ""
    except (OSError, subprocess.TimeoutExpired):
        return ""


def available() -> bool:
    return bool(os.environ.get("DISPLAY")) and os.environ.get("XDG_SESSION_TYPE", "x11") != "wayland"


def client_windows() -> list[int]:
    """Top-level windows managed by the window manager."""
    listing = _xprop("-root", "_NET_CLIENT_LIST")
    return [int(value, 16) for value in re.findall(r"0x[0-9a-fA-F]+", listing.split("#", 1)[-1])]


def window_pid(window: int) -> int | None:
    match = re.search(r"_NET_WM_PID\(CARDINAL\) = (\d+)", _xprop("-id", hex(window), "_NET_WM_PID"))
    return int(match.group(1)) if match else None


def window_title(window: int) -> str:
    match = re.search(r'WM_NAME\([^)]*\) = "(.*)"', _xprop("-id", hex(window), "WM_NAME"))
    return match.group(1) if match else ""


def find_window(pids: set[int], title_contains: str) -> int | None:
    """The client window owned by one of `pids` whose title contains
    `title_contains`. The title is required: applications often show a
    splash screen first (RViz does), which must not be mistaken for the
    main window."""
    for window in client_windows():
        if window_pid(window) in pids and title_contains in window_title(window):
            return window
    return None


def exists(window: int) -> bool:
    return window in client_windows()
