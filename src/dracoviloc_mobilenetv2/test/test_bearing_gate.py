"""BearingGate unit tests - pure logic, no ROS/CUDA."""
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'mobilenetv2'))
from processing import BearingGate

X = (1.0, 0.0, 0.0)
Y = (0.0, 1.0, 0.0)          # 90 deg from X
DIAG = (1.0, 1.0, 0.0)       # 45 deg from X
FAR = (-1.0, 0.0, 0.0)       # 180 deg from X


def test_disabled_gate_accepts_everything():
    gate = BearingGate(enabled=False)
    for direction in (X, Y, FAR, X, Y):
        assert gate.accept(direction, now=1.0)[0]


def test_first_bearing_is_accepted():
    gate = BearingGate()
    assert gate.accept(X, now=1.0) == (True, False)
    assert gate.reject_count == 0


def test_small_step_accepted_and_eases_anchor():
    gate = BearingGate()
    gate.accept(X, now=1.0)
    publish, _ = gate.accept(DIAG, now=1.5)   # 45 deg < 60 deg jump limit
    assert publish
    # Anchor moved halfway toward the accepted candidate.
    assert gate.last_dir[0] == pytest.approx(math.cos(math.radians(22.5)))
    assert gate.last_dir[1] == pytest.approx(math.sin(math.radians(22.5)))


def test_sudden_jump_rejected_until_challenger_confirms():
    gate = BearingGate(confirm=3)
    gate.accept(X, now=1.0)
    assert gate.accept(Y, now=1.5) == (False, False)
    assert gate.accept(Y, now=2.0) == (False, False)
    assert gate.accept(Y, now=2.5) == (True, True)   # retarget on 3rd
    # Anchor is the new bearing; following candidates near it pass.
    assert gate.accept(Y, now=3.0) == (True, False)


def test_jittery_challenger_never_confirms():
    gate = BearingGate(confirm=3, timeout=30.0)
    gate.accept(X, now=1.0)
    challengers = [Y, (0.0, -1.0, 0.0), (0.0, 0.0, 1.0),
                   (0.0, -1.0, 0.2), (-1.0, 0.0, -0.5)]
    for i, direction in enumerate(challengers):
        publish, _ = gate.accept(direction, now=1.5 + i * 0.5)
        assert not publish
        assert gate.reject_count <= 2   # cone resets keep it below confirm


def test_timeout_rearms_gate():
    gate = BearingGate(timeout=2.0)
    gate.accept(X, now=1.0)
    assert not gate.accept(Y, now=2.0)[0]                    # within timeout: rejected
    publish, retargeted = gate.accept(Y, now=4.0)            # timeout: re-arm, accept
    assert publish and not retargeted


def test_zero_and_nonfinite_directions_rejected():
    gate = BearingGate()
    assert not gate.accept((0.0, 0.0, 0.0), now=1.0)[0]
    assert not gate.accept((float('nan'), 1.0, 0.0), now=1.0)[0]
    assert gate.last_dir is None   # gate untouched, still unarmed


def test_unnormalized_input_normalized():
    gate = BearingGate()
    assert gate.accept((10.0, 0.0, 0.0), now=1.0)[0]
    assert gate.last_dir == (1.0, 0.0, 0.0)
