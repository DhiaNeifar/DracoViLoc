"""Embed another application's X11 window (RViz) inside the GUI window.

Qt's WindowContainer cannot take a window that the window manager already
manages, so this goes through Xlib directly: withdraw the window from the
window manager, reparent it into the GUI window, then keep moving/resizing it
over the panel that shows it. Mouse and keyboard input go straight to the
embedded application (rotate, pan and zoom work as in RViz itself).

The embedded window is a native child window, which always draws above the
QML content; callers hide it whenever something must appear on top of it.
"""
from __future__ import annotations

import ctypes
import time


class X11Embedder:
    def __init__(self) -> None:
        self._x11 = ctypes.cdll.LoadLibrary("libX11.so.6")
        self._x11.XOpenDisplay.restype = ctypes.c_void_p
        self._x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
        handle = self._x11.XOpenDisplay(None)
        if not handle:
            raise RuntimeError("cannot open the X11 display")
        self._display = ctypes.c_void_p(handle)
        self._mapped: dict[int, bool] = {}

    def _sync(self) -> None:
        self._x11.XSync(self._display, 0)

    def embed(self, child: int, parent: int, x: int, y: int, width: int, height: int) -> None:
        window = ctypes.c_ulong(child)
        self._x11.XWithdrawWindow(self._display, window, self._x11.XDefaultScreen(self._display))
        self._sync()
        time.sleep(0.3)  # let the window manager let go of it
        self._x11.XReparentWindow(self._display, window, ctypes.c_ulong(parent), x, y)
        self._x11.XMoveResizeWindow(self._display, window, x, y, max(1, width), max(1, height))
        self._x11.XMapWindow(self._display, window)
        self._mapped[child] = True
        self._sync()

    def place(self, child: int, x: int, y: int, width: int, height: int, visible: bool) -> None:
        window = ctypes.c_ulong(child)
        if visible:
            self._x11.XMoveResizeWindow(self._display, window, x, y, max(1, width), max(1, height))
            if not self._mapped.get(child):
                self._x11.XMapWindow(self._display, window)
                self._mapped[child] = True
        elif self._mapped.get(child):
            self._x11.XUnmapWindow(self._display, window)
            self._mapped[child] = False
        self._sync()

    def forget(self, child: int) -> None:
        self._mapped.pop(child, None)
