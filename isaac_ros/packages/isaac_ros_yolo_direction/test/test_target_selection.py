from types import SimpleNamespace
import unittest

from isaac_ros_yolo_direction.target_selection import DetectionLock


def box(x, y, score=0.8):
    return SimpleNamespace(
        bbox=SimpleNamespace(center=SimpleNamespace(
            position=SimpleNamespace(x=x, y=y))),
        results=[SimpleNamespace(hypothesis=SimpleNamespace(score=score))])


class DetectionLockTest(unittest.TestCase):
    def test_confirms_target_and_ignores_transient_far_box(self):
        lock = DetectionLock()
        self.assertIsNone(lock.select([box(100, 200)], 0.0))
        self.assertIsNone(lock.select([box(104, 201)], 0.067))
        self.assertIsNotNone(lock.select([box(108, 202)], 0.134))
        self.assertIsNone(lock.select([box(620, 250)], 0.201))
        self.assertIsNotNone(lock.select([box(110, 203)], 0.268))

    def test_reacquisition_needs_three_consistent_frames_after_gap(self):
        lock = DetectionLock()
        for t in (0.0, 0.067, 0.134):
            selected = lock.select([box(100, 200)], t)
        self.assertIsNotNone(selected)
        self.assertIsNone(lock.select([], 0.201))
        self.assertIsNone(lock.select([box(600, 300)], 1.0))
        self.assertIsNone(lock.select([box(602, 301)], 1.067))
        self.assertIsNotNone(lock.select([box(604, 302)], 1.134))


if __name__ == '__main__':
    unittest.main()
