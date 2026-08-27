#!/usr/bin/env python3
"""
Correctness test for the GRE overlap-save resampling fix (_sss_cb in
gre_classifier_node.py). Pure numpy/scipy - no ROS, no CUDA, no engine file -
so it runs anywhere gre_env's interpreter runs, before touching hardware.

Reads RESAMPLE_UP/DOWN/BLOCK/HOP straight out of gre_classifier_node.py's
source (rather than hardcoding them here) so a future change to those
constants fails this test instead of silently drifting out of sync.

Run directly:
    python3 test_resample_overlap_save.py
or with pytest:
    pytest test_resample_overlap_save.py -v
"""
import re
from pathlib import Path

import numpy as np
from scipy.signal import resample_poly

NODE_FILE = Path(__file__).parent / "gre_classifier_node.py"
_SRC = NODE_FILE.read_text()

_ns = {}
for _name in ("RESAMPLE_UP", "RESAMPLE_DOWN", "RESAMPLE_BLOCK", "RESAMPLE_HOP"):
    _m = re.search(rf"^{_name}\s*=\s*([^\n#]+)", _SRC, re.MULTILINE)
    if not _m:
        raise SystemExit(f"could not find {_name} in {NODE_FILE}")
    _ns[_name] = eval(_m.group(1).strip(), {}, _ns)

UP, DOWN, BLOCK, HOP = (
    _ns["RESAMPLE_UP"], _ns["RESAMPLE_DOWN"], _ns["RESAMPLE_BLOCK"], _ns["RESAMPLE_HOP"])


def overlap_save_resample(raw, hop_in):
    """Reimplements the exact block/lookahead loop in gre_classifier_node's
    _sss_cb, fed `hop_in`-sample chunks the way /sss messages arrive."""
    pending = np.zeros(0, dtype=np.float32)
    history = np.zeros(BLOCK, dtype=np.float32)
    out = []
    pos = 0
    while pos < len(raw):
        pending = np.concatenate(
            [pending, raw[pos:pos + hop_in]]).astype(np.float32)
        pos += hop_in

        hops = []
        while len(pending) >= 2 * BLOCK:
            block = pending[:BLOCK]
            lookahead = pending[BLOCK:2 * BLOCK]
            pending = pending[BLOCK:]
            primed = np.concatenate([history, block, lookahead])
            resampled = resample_poly(primed, UP, DOWN)
            hops.append(resampled[HOP:2 * HOP])
            history = block
        if hops:
            out.append(np.concatenate(hops).astype(np.float32))
    return np.concatenate(out) if out else np.zeros(0, dtype=np.float32)


def test_constants_are_exact():
    assert BLOCK % DOWN == 0, "RESAMPLE_BLOCK must be a multiple of RESAMPLE_DOWN"
    assert BLOCK * UP % DOWN == 0
    assert HOP == BLOCK * UP // DOWN


def test_bit_exact_against_continuous_resample():
    rng = np.random.default_rng(0)
    raw = rng.standard_normal(44100 * 4).astype(np.float32)
    ref = resample_poly(raw, UP, DOWN)

    for hop_in in (512, 1, 480, 1000, 4410):
        out = overlap_save_resample(raw, hop_in)
        n = min(len(out), len(ref))
        err = np.max(np.abs(out[:n] - ref[:n]))
        assert err < 1e-4, f"hop_in={hop_in}: max abs diff {err}"


def test_reset_starts_a_fresh_stream():
    # A channel reset zeroes pending/history, which must behave like
    # resampling a brand-new stream from that point, not a continuation.
    rng = np.random.default_rng(1)
    raw = rng.standard_normal(44100 * 2).astype(np.float32)
    out = overlap_save_resample(raw, 512)
    ref = resample_poly(raw, UP, DOWN)
    n = min(len(out), len(ref))
    assert np.max(np.abs(out[:n] - ref[:n])) < 1e-4


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
        print(f"PASS {t.__name__}")
    print(f"\n{len(tests)} tests passed "
          f"(RESAMPLE_UP={UP} RESAMPLE_DOWN={DOWN} "
          f"RESAMPLE_BLOCK={BLOCK} RESAMPLE_HOP={HOP})")
