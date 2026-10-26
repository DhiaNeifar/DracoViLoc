import math
from types import SimpleNamespace
import unittest

from isaac_ros_yolo_direction.direction_publisher import (
    map_detection_pixel_to_raw, raw_pixel_to_ray_slopes)


RAW = SimpleNamespace(k=[848.0, 0.0, 424.0, 0.0, 480.0, 240.0, 0.0, 0.0, 1.0])


def encoded_info(letterbox):
    sx = 640.0 / 848.0
    sy = sx if letterbox else 640.0 / 480.0
    pad_y = (640.0 - 480.0 * sy) / 2.0
    return SimpleNamespace(k=[
        RAW.k[0] * sx, 0.0, RAW.k[2] * sx,
        0.0, RAW.k[4] * sy, RAW.k[5] * sy + pad_y,
        0.0, 0.0, 1.0])


def encode(raw_u, raw_v, encoded):
    return (
        (raw_u - RAW.k[2]) * encoded.k[0] / RAW.k[0] + encoded.k[2],
        (raw_v - RAW.k[5]) * encoded.k[4] / RAW.k[4] + encoded.k[5])


class DirectionGeometryTest(unittest.TestCase):
    def test_letterbox_and_stretch_round_trip(self):
        for letterbox in (True, False):
            encoded = encoded_info(letterbox)
            for raw_v in (0.0, 240.0, 479.0):
                with self.subTest(letterbox=letterbox, raw_v=raw_v):
                    u, v = encode(424.0, raw_v, encoded)
                    raw_u, mapped_v = map_detection_pixel_to_raw(u, v, RAW, encoded)
                    _, vertical = raw_pixel_to_ray_slopes(raw_u, mapped_v, RAW)
                    actual = math.degrees(math.atan(vertical))
                    expected = math.degrees(math.atan((raw_v - RAW.k[5]) / RAW.k[4]))
                    self.assertAlmostEqual(raw_u, 424.0, delta=1.0)
                    self.assertAlmostEqual(mapped_v, raw_v, delta=1.0)
                    self.assertAlmostEqual(actual, expected, delta=0.1)


if __name__ == '__main__':
    unittest.main()
