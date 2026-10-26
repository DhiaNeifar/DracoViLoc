"""Keep a visual target through short gaps without following isolated box jumps."""

import math


class DetectionLock:
    def __init__(self, max_step_px=120.0, confirmation_frames=3,
                 lost_timeout_s=0.75):
        self.max_step_px = float(max_step_px)
        self.confirmation_frames = int(confirmation_frames)
        self.lost_timeout_s = float(lost_timeout_s)
        self.center = None
        self.last_seen = None
        self.pending_center = None
        self.pending_count = 0
        self.pending_stamp = None

    @staticmethod
    def _center(detection):
        point = detection.bbox.center.position
        return (point.x, point.y)

    @staticmethod
    def _score(detection):
        return detection.results[0].hypothesis.score

    def _near(self, first, second):
        return math.dist(first, second) <= self.max_step_px

    def _clear_pending(self):
        self.pending_center = None
        self.pending_count = 0
        self.pending_stamp = None

    def select(self, detections, stamp_s):
        """Return a confirmed box, or None while the target is absent/uncertain."""
        if self.last_seen is not None and stamp_s - self.last_seen > self.lost_timeout_s:
            self.center = None

        valid = [d for d in detections if d.results]
        if not valid:
            self._clear_pending()
            return None

        if self.center is not None:
            nearby = [d for d in valid if self._near(self.center, self._center(d))]
            if nearby:
                chosen = max(nearby, key=self._score)
                self.center = self._center(chosen)
                self.last_seen = stamp_s
                self._clear_pending()
                return chosen

        chosen = max(valid, key=self._score)
        candidate = self._center(chosen)
        consecutive = (
            self.pending_center is not None
            and self._near(self.pending_center, candidate)
            and self.pending_stamp is not None
            and 0.0 <= stamp_s - self.pending_stamp <= 0.2
        )
        self.pending_count = self.pending_count + 1 if consecutive else 1
        self.pending_center = candidate
        self.pending_stamp = stamp_s

        if self.pending_count < self.confirmation_frames:
            return None
        if self.center is not None and self.last_seen is not None and (
                stamp_s - self.last_seen < self.lost_timeout_s):
            return None

        self.center = candidate
        self.last_seen = stamp_s
        self._clear_pending()
        return chosen
