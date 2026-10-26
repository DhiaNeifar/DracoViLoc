from __future__ import annotations

from PySide6.QtGui import QImage
from PySide6.QtQuick import QQuickImageProvider


class CameraImageProvider(QQuickImageProvider):
    """Latest frame per stream ("raw", "overlay"); QML asks for "<stream>?<rev>"."""

    def __init__(self) -> None:
        super().__init__(QQuickImageProvider.Image)
        self.images: dict[str, QImage] = {}

    def update(self, frame: dict) -> None:
        stream = frame.get("stream", "overlay")
        if frame.get("encoding") == "jpeg":
            image = QImage.fromData(frame["data"], "JPEG")  # decoded in C++
        else:
            image = QImage(frame["data"], frame["width"], frame["height"], frame["step"],
                           QImage.Format_RGB888).copy()
            if frame["encoding"] == "bgr8":
                image = image.rgbSwapped()
        if not image.isNull():
            self.images[stream] = image

    def requestImage(self, image_id, size, requested_size):
        image = self.images.get(image_id.split("?", 1)[0], QImage())
        if requested_size.width() > 0 and requested_size.height() > 0 and not image.isNull():
            image = image.scaled(requested_size)
        size.setWidth(image.width())
        size.setHeight(image.height())
        return image
