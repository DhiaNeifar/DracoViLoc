from __future__ import annotations

from PySide6.QtCore import Signal, QObject
from PySide6.QtGui import QImage
from PySide6.QtQuick import QQuickImageProvider


class CameraImageProvider(QQuickImageProvider):
    def __init__(self) -> None:
        super().__init__(QQuickImageProvider.Image)
        self.image = QImage()

    def update(self, frame: dict) -> None:
        fmt = QImage.Format_RGB888
        image = QImage(
            frame["data"], frame["width"], frame["height"], frame["step"], fmt).copy()
        if frame["encoding"] == "bgr8":
            image = image.rgbSwapped()
        self.image = image

    def requestImage(self, _id, size, requested_size):
        image = self.image
        if requested_size.width() > 0 and requested_size.height() > 0 and not image.isNull():
            image = image.scaled(requested_size)
        size.setWidth(image.width())
        size.setHeight(image.height())
        return image

