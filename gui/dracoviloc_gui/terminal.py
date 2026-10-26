"""Terminal panels: per-process scrollback with ANSI colors, for QML."""
from __future__ import annotations

import html
import re

from PySide6.QtCore import QAbstractListModel, QModelIndex, QObject, Qt, Signal, Slot

MAX_LINES = 5000
TIMESTAMP = re.compile(r"^(\d{4}-\d{2}-\d{2}T(\d{2}:\d{2}:\d{2}\.\d{3})\S*) (.*)$")
ESCAPE = re.compile(r"\x1b\[([0-9;?]*)([A-Za-z])|\x1b[()][A-Za-z0-9]|\x1b[=>]")
COLORS = {
    30: "#5C6B78", 31: "#FF6B76", 32: "#5BD08A", 33: "#E8C468",
    34: "#6CA7FF", 35: "#D78BF0", 36: "#57CAD3", 37: "#C8D7E0",
    90: "#7E8F9C", 91: "#FF8992", 92: "#7FE3A8", 93: "#F2D98A",
    94: "#93BFFF", 95: "#E3A8F5", 96: "#8BE3EA", 97: "#FFFFFF",
}
ERROR_HINTS = ("[ERROR]", "[FATAL]", "Traceback", "Error:", "error:", "FAILED", "failed with")
WARNING_HINTS = ("[WARN]", "WARNING", "Warning:")


def severity_of(text: str) -> str:
    if any(hint in text for hint in ERROR_HINTS):
        return "error"
    if any(hint in text for hint in WARNING_HINTS):
        return "warning"
    return "info"


def split_line(line: str) -> tuple[str, str]:
    """Split the wrapper's 'ISO-timestamp text' line into (HH:MM:SS.mmm, text)."""
    match = TIMESTAMP.match(line)
    if match:
        return match.group(2), match.group(3)
    return "", line


def strip_ansi(text: str) -> str:
    return ESCAPE.sub("", text.rsplit("\r", 1)[-1])


def ansi_to_styled(text: str) -> str:
    """Convert SGR color codes to Qt StyledText; drop other escape sequences."""
    text = text.rsplit("\r", 1)[-1]  # progress bars: keep the last redraw only
    out: list[str] = []
    color: str | None = None
    bold = False
    position = 0

    def open_tags() -> None:
        if color:
            out.append(f'<font color="{color}">')
        if bold:
            out.append("<b>")

    def close_tags() -> None:
        if bold:
            out.append("</b>")
        if color:
            out.append("</font>")

    for match in ESCAPE.finditer(text):
        out.append(html.escape(text[position:match.start()]))
        position = match.end()
        if match.group(2) != "m":
            continue
        close_tags()
        codes = [int(code) for code in (match.group(1) or "0").split(";") if code.isdigit()] or [0]
        index = 0
        while index < len(codes):
            code = codes[index]
            if code == 0:
                color, bold = None, False
            elif code == 1:
                bold = True
            elif code == 22:
                bold = False
            elif code == 39:
                color = None
            elif code in COLORS:
                color = COLORS[code]
            elif code == 38 and index + 2 < len(codes) and codes[index + 1] == 5:
                color = COLORS.get(30 + codes[index + 2] % 8) if codes[index + 2] < 8 else (
                    COLORS.get(90 + codes[index + 2] - 8) if codes[index + 2] < 16 else color)
                index += 2
            index += 1
        open_tags()
    out.append(html.escape(text[position:]))
    close_tags()
    return "".join(out)


class TerminalModel(QAbstractListModel):
    TimeRole = Qt.UserRole + 1
    HtmlRole = Qt.UserRole + 2
    SeverityRole = Qt.UserRole + 3

    def __init__(self, name: str, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self.name = name
        self._rows: list[tuple[str, str, str]] = []

    def roleNames(self):
        return {self.TimeRole: b"time", self.HtmlRole: b"html", self.SeverityRole: b"severity"}

    def rowCount(self, parent=QModelIndex()) -> int:
        return 0 if parent.isValid() else len(self._rows)

    def data(self, index, role=Qt.DisplayRole):
        if not index.isValid():
            return None
        time_text, styled, severity = self._rows[index.row()]
        return {self.TimeRole: time_text, self.HtmlRole: styled,
                self.SeverityRole: severity, Qt.DisplayRole: styled}.get(role)

    def append_lines(self, lines: list[str]) -> None:
        rows = []
        for line in lines:
            time_text, text = split_line(line)
            rows.append((time_text, ansi_to_styled(text), severity_of(strip_ansi(text))))
        overflow = len(self._rows) + len(rows) - MAX_LINES
        if overflow > 0:
            drop = min(overflow, len(self._rows))
            self.beginRemoveRows(QModelIndex(), 0, drop - 1)
            del self._rows[:drop]
            self.endRemoveRows()
            rows = rows[-MAX_LINES:]
        self.beginInsertRows(QModelIndex(), len(self._rows), len(self._rows) + len(rows) - 1)
        self._rows.extend(rows)
        self.endInsertRows()

    @Slot()
    def clear(self) -> None:
        self.beginResetModel()
        self._rows.clear()
        self.endResetModel()


class TerminalHub(QObject):
    """Receives lines from worker threads and appends them on the GUI thread."""

    linesArrived = Signal(str, list)
    terminalsChanged = Signal()

    def __init__(self, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self._models: dict[str, TerminalModel] = {}
        self.linesArrived.connect(self._append, Qt.QueuedConnection)

    def names(self) -> list[str]:
        return list(self._models)

    def model(self, name: str) -> TerminalModel:
        if name not in self._models:
            self._models[name] = TerminalModel(name, self)
            self.terminalsChanged.emit()
        return self._models[name]

    def _append(self, name: str, lines: list) -> None:
        self.model(name).append_lines(lines)
