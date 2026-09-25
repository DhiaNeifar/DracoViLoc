"""Tests for deterministic SSS/SST ordinal pairing (report section 7)."""

import math

import pytest

from stub_ros import lib_odas_server_node

PairCoordinator = lib_odas_server_node.PairCoordinator

SLOT_COUNT = 4
HOP_DURATION = 512 / 44100
BASELINE = 1000


def make_snapshot(slots):
    return [
        {'id': track_id, 'x': 1.0, 'y': 2.0, 'z': 3.0, 'activity': 0.5}
        for track_id in slots
    ]


def make_frame(ordinal):
    return bytes([ordinal % 251]) * 16


class LogRecorder:
    def __init__(self):
        self.infos = []
        self.warnings = []
        self.errors = []

    def info(self, msg):
        self.infos.append(msg)

    def warning(self, msg):
        self.warnings.append(msg)

    def error(self, msg):
        self.errors.append(msg)


def make_coordinator(**overrides):
    recorder = LogRecorder()
    kwargs = {
        'slot_count': SLOT_COUNT,
        'hop_duration': HOP_DURATION,
        'log_info': recorder.info,
        'log_warn': recorder.warning,
        'log_error': recorder.error,
    }
    kwargs.update(overrides)
    return PairCoordinator(**kwargs), recorder


def start_session(coordinator):
    """Connect both streams; returns (sss_token, sst_token)."""
    sss_token, sss_events = coordinator.register_connection('sss')
    assert coordinator.is_session_active() is False
    sst_token, sst_events = coordinator.register_connection('sst')
    assert coordinator.is_session_active() is True
    assert any(e['type'] == 'session_started' for e in sst_events)
    return sss_token, sst_token


def submit_sss(coordinator, token, ordinal, recv_time):
    return coordinator.submit_sss_frame(token, make_frame(ordinal), recv_time)


def submit_sst(coordinator, token, time_stamp, recv_time, slots=(7, 0, 0, 0)):
    return coordinator.submit_sst_snapshot(token, time_stamp, make_snapshot(slots), recv_time)


def completed_pairs(events):
    return [e for e in events if e['type'] == 'pair']


def violations(events):
    return [e for e in events if e['type'] == 'violation']


class TestOrdinalPairing:
    def test_sss_ordinal_k_pairs_with_baseline_plus_k_minus_1(self):
        coordinator, _ = make_coordinator()
        sss_token, sst_token = start_session(coordinator)

        pairs = []
        for k in range(1, 4):
            events = submit_sss(coordinator, sss_token, k, recv_time=10.0 + k * HOP_DURATION)
            assert completed_pairs(events) == []
            events = submit_sst(coordinator, sst_token, BASELINE + k - 1,
                                recv_time=10.1 + k * HOP_DURATION)
            pairs += completed_pairs(events)

        assert [p['ordinal'] for p in pairs] == [1, 2, 3]
        assert pairs[0]['sss_data'] == make_frame(1)
        assert [s['id'] for s in pairs[0]['sst_snapshot']] == [7, 0, 0, 0]
        # Session epoch is anchored at the first completed pair; the stamp is a
        # sample-time sequence: epoch + (ordinal - 1) * hop_duration.
        epoch = max(10.0 + HOP_DURATION, 10.1 + HOP_DURATION)
        for pair in pairs:
            assert pair['stamp'] == pytest.approx(epoch + (pair['ordinal'] - 1) * HOP_DURATION)

    def test_sst_arriving_before_sss_pairs_identically(self):
        coordinator, _ = make_coordinator()
        sss_token, sst_token = start_session(coordinator)

        submit_sst(coordinator, sst_token, BASELINE, recv_time=5.0)
        events = submit_sss(coordinator, sss_token, 1, recv_time=5.2)

        pairs = completed_pairs(events)
        assert len(pairs) == 1
        assert pairs[0]['ordinal'] == 1

    def test_delayed_matching_ordinal_pairs(self):
        coordinator, _ = make_coordinator()
        sss_token, sst_token = start_session(coordinator)

        # SSS runs ahead; SST catches up later.
        submit_sss(coordinator, sss_token, 1, recv_time=0.0)
        submit_sss(coordinator, sss_token, 2, recv_time=HOP_DURATION)
        assert completed_pairs(submit_sst(coordinator, sst_token, BASELINE, recv_time=0.4)) != []
        assert completed_pairs(submit_sst(coordinator, sst_token, BASELINE + 1,
                                          recv_time=0.4 + HOP_DURATION)) != []

    def test_ordinal_never_published_twice(self):
        coordinator, _ = make_coordinator()
        sss_token, sst_token = start_session(coordinator)

        submit_sss(coordinator, sss_token, 1, recv_time=0.0)
        first = completed_pairs(submit_sst(coordinator, sst_token, BASELINE, recv_time=0.1))
        assert len(first) == 1
        # A second submit with the same data cannot re-trigger the pair.
        assert completed_pairs(submit_sss(coordinator, sss_token, 2, recv_time=0.2)) == []
        assert completed_pairs(submit_sst(coordinator, sst_token, BASELINE + 1,
                                          recv_time=0.3)) != []

    def test_successful_pair_resets_consecutive_violation_counter(self):
        coordinator, _ = make_coordinator()
        sss_token, sst_token = start_session(coordinator)

        # Validate the session with one clean pair.
        submit_sss(coordinator, sss_token, 1, recv_time=0.0)
        submit_sst(coordinator, sst_token, BASELINE, recv_time=0.1)

        # One violation, then a successful pair resets the counter.
        events = submit_sst(coordinator, sst_token, BASELINE, recv_time=0.2)
        assert len(violations(events)) == 1
        submit_sss(coordinator, sss_token, 2, recv_time=0.3)
        submit_sst(coordinator, sst_token, BASELINE + 1, recv_time=0.4)
        assert completed_pairs(events) == []

        events = submit_sst(coordinator, sst_token, BASELINE + 1, recv_time=0.5)
        new_violations = violations(events)
        assert len(new_violations) == 1
        assert new_violations[0]['consecutive_violations'] == 1


class TestStartupValidation:
    def test_non_integer_first_timestamp_disables_paired_mode(self):
        coordinator, recorder = make_coordinator()
        sss_token, sst_token = start_session(coordinator)

        events = submit_sst(coordinator, sst_token, 12.5, recv_time=0.0)

        assert any(e['type'] == 'paired_mode_disabled' for e in events)
        assert coordinator.is_paired_enabled() is False
        assert any('not an integer' in m for m in recorder.errors)
        # Everything afterwards is ignored.
        assert submit_sss(coordinator, sss_token, 1, recv_time=0.1) == []

    @pytest.mark.parametrize('bad_stamp', [True, '1000', None])
    def test_non_integer_types_rejected_as_baseline(self, bad_stamp):
        coordinator, _ = make_coordinator()
        _, sst_token = start_session(coordinator)

        events = submit_sst(coordinator, sst_token, bad_stamp, recv_time=0.0)

        assert any(e['type'] == 'paired_mode_disabled' for e in events)

    def test_timestamp_must_increment_by_one_before_validation(self):
        coordinator, recorder = make_coordinator()
        sss_token, sst_token = start_session(coordinator)

        submit_sst(coordinator, sst_token, BASELINE, recv_time=0.0)
        events = submit_sst(coordinator, sst_token, BASELINE + 2, recv_time=0.1)

        assert any(e['type'] == 'paired_mode_disabled' for e in events)
        assert coordinator.is_paired_enabled() is False
        assert any('increments by exactly 1' in m for m in recorder.errors)
        assert submit_sss(coordinator, sss_token, 1, recv_time=0.2) == []

    def test_invalid_hop_duration_rejected(self):
        recorder = LogRecorder()
        with pytest.raises(ValueError):
            PairCoordinator(slot_count=SLOT_COUNT, hop_duration=0.0,
                            log_info=recorder.info, log_warn=recorder.warning,
                            log_error=recorder.error)
        with pytest.raises(ValueError):
            PairCoordinator(slot_count=SLOT_COUNT, hop_duration=math.inf,
                            log_info=recorder.info, log_warn=recorder.warning,
                            log_error=recorder.error)


class TestViolationsTwoTierPolicy:
    def _validated_session(self, **kwargs):
        coordinator, recorder = make_coordinator(**kwargs)
        sss_token, sst_token = start_session(coordinator)
        submit_sss(coordinator, sss_token, 1, recv_time=0.0)
        submit_sst(coordinator, sst_token, BASELINE, recv_time=0.1)
        return coordinator, recorder, sss_token, sst_token

    def test_duplicate_ordinal_is_dropped_and_counted(self):
        coordinator, _, sss_token, sst_token = self._validated_session()

        submit_sss(coordinator, sss_token, 2, recv_time=0.2)
        events = submit_sst(coordinator, sst_token, BASELINE + 1, recv_time=0.25)
        assert completed_pairs(events) != []
        # Exact duplicate of the last timestamp: dropped, no pairing shift.
        events = submit_sst(coordinator, sst_token, BASELINE + 1, recv_time=0.3)

        dup = [v for v in violations(events) if v['kind'] == 'duplicate_ordinal']
        assert len(dup) == 1
        assert dup[0]['consecutive_violations'] == 1
        assert completed_pairs(events) == []

    def test_decreasing_ordinal_is_dropped_and_counted(self):
        coordinator, _, sss_token, sst_token = self._validated_session()

        events = submit_sst(coordinator, sst_token, BASELINE - 1, recv_time=0.2)

        dup = [v for v in violations(events) if v['kind'] == 'duplicate_ordinal']
        assert len(dup) == 1

    def test_ordinal_gap_drains_only_affected_window(self):
        coordinator, _, sss_token, sst_token = self._validated_session()

        # SSS frames 2 and 3 arrive; SST skips timestamp BASELINE+1.
        submit_sss(coordinator, sss_token, 2, recv_time=0.2)
        submit_sss(coordinator, sss_token, 3, recv_time=0.3)
        events = submit_sst(coordinator, sst_token, BASELINE + 2, recv_time=0.35)

        gap = [v for v in violations(events) if v['kind'] == 'ordinal_gap']
        assert len(gap) == 1
        # Ordinal 2's SSS frame lost its SST counterpart and was drained;
        # ordinal 3 pairs immediately (SSS frame 3 was already pending) and
        # ordinals never shift.
        pairs = completed_pairs(events)
        assert [p['ordinal'] for p in pairs] == [3]
        assert pairs[0]['sss_data'] == make_frame(3)
        assert completed_pairs(submit_sss(coordinator, sss_token, 4, recv_time=0.4)) == []
        pairs = completed_pairs(submit_sst(coordinator, sst_token, BASELINE + 3,
                                           recv_time=0.45))
        assert [p['ordinal'] for p in pairs] == [4]

    def test_pending_overflow_drains_oldest_and_counts_violations(self):
        coordinator, _, sss_token, sst_token = self._validated_session(
            max_pending_pairs=4)

        events = []
        for k in range(2, 8):
            events += submit_sst(coordinator, sst_token, BASELINE + k - 1, recv_time=0.1 * k)

        overflow = [v for v in events if v['kind'] == 'pending_overflow']
        # 4 kept + 2 drained beyond the bound.
        assert len(overflow) == 2
        assert overflow[0]['consecutive_violations'] == 1
        assert overflow[1]['consecutive_violations'] == 2

    def test_pair_timeout_violation_and_drain(self):
        coordinator, _, sss_token, sst_token = self._validated_session(
            session_invalidate_timeout=0.5)

        # Ordinal 2 SSS frame arrives then ages out before its SST counterpart.
        submit_sss(coordinator, sss_token, 2, recv_time=0.20)
        events = submit_sst(coordinator, sst_token, BASELINE + 1, recv_time=0.80)

        timeout = [v for v in events if v['kind'] == 'pair_timeout']
        assert len(timeout) == 1
        assert completed_pairs(events) == []
        # The next SSS frame pairs with the next SST snapshot by identical
        # ordinal: ordinal associations never shift after a drain.
        assert completed_pairs(submit_sss(coordinator, sss_token, 3, recv_time=0.90)) == []
        pairs = completed_pairs(submit_sst(coordinator, sst_token, BASELINE + 2,
                                           recv_time=0.95))
        assert [p['ordinal'] for p in pairs] == [3]

    def test_three_consecutive_violations_invalidate_session(self):
        coordinator, recorder, sss_token, sst_token = self._validated_session(
            max_consecutive_pair_violations=3)

        invalidated = []
        for _ in range(3):
            events = submit_sst(coordinator, sst_token, BASELINE, recv_time=1.0)
            invalidated += [e for e in events if e['type'] == 'session_invalidated']

        assert len(invalidated) == 1
        assert coordinator.is_session_active() is False
        assert any('consecutive pairing violations' in m for m in recorder.errors)
        # Data from invalidated tokens is ignored.
        assert submit_sss(coordinator, sss_token, 2, recv_time=1.1) == []
        assert submit_sst(coordinator, sst_token, BASELINE + 1, recv_time=1.2) == []


class TestSessionLifecycle:
    def test_disconnect_invalidates_session_immediately(self):
        coordinator, _, sss_token, sst_token = start_session_coordinator()

        events = coordinator.connection_lost(sss_token)

        invalidated = [e for e in events if e['type'] == 'session_invalidated']
        assert len(invalidated) == 1
        assert 'disconnect' in invalidated[0]['reason']
        assert coordinator.is_session_active() is False
        # Stale-token data is ignored.
        assert submit_sst(coordinator, sst_token, BASELINE, recv_time=0.1) == []

    def test_stale_token_disconnect_is_ignored(self):
        coordinator, _ = make_coordinator()
        sss_token, sst_token = start_session(coordinator)
        coordinator.connection_lost(sss_token)

        assert coordinator.connection_lost(sss_token) == []
        assert coordinator.connection_lost(sst_token) == []

    def test_cross_session_data_never_pairs(self):
        coordinator, _ = make_coordinator()
        sss_token, sst_token = start_session(coordinator)
        submit_sss(coordinator, sss_token, 1, recv_time=0.0)
        coordinator.connection_lost(sss_token)

        # New session with a new baseline.
        sss_token2, sst_token2 = start_session(coordinator)
        submit_sst(coordinator, sst_token2, 5000, recv_time=1.0)
        # Old SSS frame from the invalidated session must not pair with the
        # new session's SST snapshot, even though both are ordinal 1.
        assert completed_pairs(submit_sss(coordinator, sss_token, 1, recv_time=1.1)) == []
        pairs = completed_pairs(submit_sss(coordinator, sss_token2, 1, recv_time=1.2))
        assert [p['ordinal'] for p in pairs] == [1]
        assert [s['id'] for s in pairs[0]['sst_snapshot']] == [7, 0, 0, 0]

    def test_restart_resets_sss_ordinal_and_baseline(self):
        coordinator, recorder, sss_token, sst_token = start_session_coordinator()
        submit_sss(coordinator, sss_token, 1, recv_time=0.0)
        submit_sst(coordinator, sst_token, BASELINE, recv_time=0.1)
        coordinator.connection_lost(sst_token)

        sss_token2, sst_token2 = start_session(coordinator)
        assert coordinator.is_session_active() is True
        # New session: SSS ordinal restarts at 1, baseline from the new first SST.
        submit_sss(coordinator, sss_token2, 1, recv_time=1.0)
        pairs = completed_pairs(submit_sst(coordinator, sst_token2, 7000, recv_time=1.1))
        assert [p['ordinal'] for p in pairs] == [1]
        assert any('baseline recorded: timeStamp 7000' in m for m in recorder.infos)


class TestTimestampSanity:
    def test_receive_time_skew_warns_but_pairs(self):
        coordinator, recorder, sss_token, sst_token = start_session_coordinator()

        # 0.3 s skew exceeds sst_timeout (0.25 s) but stays inside the
        # session_invalidate_timeout window, so the pair is still published.
        events = submit_sss(coordinator, sss_token, 1, recv_time=0.90)
        assert events == []
        events = submit_sst(coordinator, sst_token, BASELINE, recv_time=1.20)

        skew = [e for e in events if e['type'] == 'pair_timestamp_skew']
        assert len(skew) == 1
        assert skew[0]['skew'] == pytest.approx(0.3)
        pairs = completed_pairs(events)
        assert [p['ordinal'] for p in pairs] == [1]
        assert any('skew' in m for m in recorder.warnings)

    def test_skew_warning_throttled(self):
        coordinator, recorder, sss_token, sst_token = start_session_coordinator()
        now = [10.0]
        coordinator._clock = lambda: now[0]

        for k in range(1, 4):
            submit_sss(coordinator, sss_token, k, recv_time=now[0])
            submit_sst(coordinator, sst_token, BASELINE + k - 1, recv_time=now[0] + 0.3)
            now[0] += HOP_DURATION

        assert len(recorder.warnings) == 1
        now[0] += 2.0
        submit_sss(coordinator, sss_token, 4, recv_time=now[0])
        submit_sst(coordinator, sst_token, BASELINE + 3, recv_time=now[0] + 0.3)
        assert len(recorder.warnings) == 2


class TestDiagnostics:
    def test_baseline_and_first_pair_logged(self):
        coordinator, recorder, sss_token, sst_token = start_session_coordinator()

        submit_sst(coordinator, sst_token, BASELINE, recv_time=0.0)
        submit_sss(coordinator, sss_token, 1, recv_time=0.1)

        assert any('SST baseline recorded: timeStamp 1000' in m for m in recorder.infos)
        assert any('First paired SSS/SST frame: ordinal 1' in m for m in recorder.infos)

    def test_violations_logged_with_kind(self):
        coordinator, recorder, sss_token, sst_token = start_session_coordinator()
        submit_sss(coordinator, sss_token, 1, recv_time=0.0)
        submit_sst(coordinator, sst_token, BASELINE, recv_time=0.1)

        submit_sst(coordinator, sst_token, BASELINE, recv_time=0.2)

        assert any('duplicate_ordinal' in m for m in recorder.errors)


def start_session_coordinator(**kwargs):
    coordinator, recorder = make_coordinator(**kwargs)
    sss_token, sst_token = start_session(coordinator)
    return coordinator, recorder, sss_token, sst_token
