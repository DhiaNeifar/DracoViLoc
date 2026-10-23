from __future__ import annotations

import os
import signal
import sys
from pathlib import Path

from PySide6.QtCore import QTimer, QUrl, qInstallMessageHandler
from PySide6.QtGui import QFontDatabase, QGuiApplication, QIcon
from PySide6.QtQml import QQmlApplicationEngine

from .controller import AppController
from .image_provider import CameraImageProvider


def project_root() -> Path:
    configured = os.environ.get("DRACOVILOC_ROOT")
    if configured:
        return Path(configured).resolve()
    return Path(__file__).resolve().parents[2]


def main() -> int:
    closing = {"value": False}

    def qt_message_handler(_mode, _context, message: str) -> None:
        teardown_noise = (
            "Cannot read property" in message or
            "Unable to assign [undefined]" in message or
            "probably intended to be a signal handler" in message
        )
        if not (closing["value"] and teardown_noise):
            print(message, file=sys.stderr)

    qInstallMessageHandler(qt_message_handler)
    app = QGuiApplication(sys.argv)
    app.setOrganizationName("DracoViLoc")
    app.setApplicationName("DracoViLoc Control Center")

    root = project_root()
    app.setWindowIcon(QIcon(str(root / "gui/assets/dracoviloc-logo.png")))
    engine = QQmlApplicationEngine()
    controller = AppController(root)
    camera = CameraImageProvider()
    engine.addImageProvider("camera", camera)
    engine.setInitialProperties({"appController": controller})

    frame_revision = {"value": 0}
    def update_camera(frame):
        camera.update(frame)
        frame_revision["value"] += 1
        for window in engine.rootObjects():
            window.setProperty("cameraRevision", frame_revision["value"])
    controller.cameraFrame.connect(update_camera)
    def quit_application():
        closing["value"] = True
        for window in engine.rootObjects():
            window.setProperty("shutdownApproved", True)
            window.setVisible(False)
        app.quit()
    controller.quitReady.connect(quit_application)

    signal.signal(signal.SIGINT, lambda *_: controller.requestQuit())
    signal.signal(signal.SIGTERM, lambda *_: controller.requestQuit())
    signal_timer = QTimer()
    signal_timer.setInterval(250)
    signal_timer.timeout.connect(lambda: None)
    signal_timer.start()

    qml = root / "gui/qml/Main.qml"
    engine.load(QUrl.fromLocalFile(str(qml)))
    if not engine.rootObjects():
        return 2
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())

