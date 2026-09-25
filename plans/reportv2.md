# DracoViLoc Audio Pipeline Corrective Implementation Plan

## 1. Executive Summary

Implement five focused corrections without changing wire formats, ROS message schemas, SST thresholds, MobileNetV2 confidence, microphone configuration, or ODAS localization algorithms:

1. Fix ALSA operator precedence and accumulate short reads until one complete interleaved microphone hop is captured.
2. Make ODAS TCP senders transmit complete buffers, suppress SIGPIPE with `MSG_NOSIGNAL`, and reconstruct PCM frames from the TCP byte stream.
3. Preserve fixed SST slots in `/sst` so `/sst.sources[N]` always describes `/sss` channel N.
4. Prevent multiple SST candidates from reserving the same free track slot.
5. Pair SSS and SST deterministically by ODAS-hop ordinal inside the ROS bridge. Timestamps become diagnostics and a downstream correlation token, never a nearest-time matching mechanism.

The existing deliberate ODAS exit on socket failure remains. In-process reconnect and ALSA XRUN recovery remain out of scope.

## 2. Current Data Flow

```text
ALSA hw:2,0
  ↓ 16-channel, interleaved, signed 32-bit, 44.1 kHz, 512-frame hop
ODAS SSL
  ↓
ODAS SST fixed track slots
  ├─ SST JSON with ODAS timeStamp → TCP 9000 → /sst
  ↓
ODAS SSS fixed separated channels
  ↓ 4-channel, signed 16-bit, 44.1 kHz, 512-frame hop
4096-byte PCM frame → TCP 10000 → /sss
  ↓
MobileNetV2
```

ODAS already verifies that the spectra, powers, and SST track messages used by SSS have identical internal timestamps. That makes strict one-to-one ordinal pairing valid for the current equal-rate configuration.

## 3. ALSA Capture Fix

### Root cause

File: `src/odas_ros/src/odas/src/source/src_hops.c`  
Function: `src_hops_process_interface_soundcard()`

Current code assigns the boolean result of `snd_pcm_readi(...) > 0` to `err`. It also treats any positive short read as a complete hop, leaving the remainder of the capture buffer stale.

All downstream converters assume a complete `hopSize × nChannels` buffer.

### Implementation

Replace the single read with a full-hop loop:

1. Use `snd_pcm_sframes_t` for the ALSA return value and accumulated frame count.
2. Continue until exactly `obj->hopSize` frames have been captured.
3. Call `snd_pcm_readi()` with:
   - Destination offset by `frames_captured × nChannels × bytes_per_sample`.
   - Remaining frame count `hopSize - frames_captured`.
4. Advance only by complete interleaved frames returned by ALSA.
5. Return success only when `frames_captured == hopSize`.
6. Treat a zero return as failure to avoid an infinite loop.
7. Treat a negative return as failure without processing the partially filled buffer.
8. Do not add `snd_pcm_recover()`, `snd_pcm_prepare()`, or other XRUN recovery.

Derive bytes per sample consistently from the configured format or:

```text
bufferSize / (hopSize × nChannels)
```

Add overflow/assertion checks ensuring this division is exact.

### Preserve existing behavior

`src_hops_process()` currently converts the previous complete buffer before capturing the next one. Do not reorder that pipeline. With the new loop, only a complete fresh hop can become the next processed buffer. If an error occurs after a partial read, the source thread terminates before that partial buffer is converted.

### Risks

- A short read may block for additional data instead of returning immediately.
- Any ALSA error, including XRUN, can still terminate capture.
- The first processed hop retains the existing initialized-silence behavior.

### Tests

Add a C unit test using a wrapped/mock `snd_pcm_readi()`:

- Full 512-frame read.
- `200 + 312` short reads.
- `100 + 100 + 312` short reads.
- Negative error before any frames.
- Negative error after a partial read.
- Zero return.
- Verify destination offsets, interleaved channel alignment, and untouched canary bytes beyond the requested buffer.

## 4. TCP Framing, Complete Sends, SIGPIPE, and Accumulator

### Sender root cause

Files/functions:

- `snk_hops.c` — `snk_hops_process_interface_socket()`
- `snk_tracks.c` — `snk_tracks_process_interface_socket()`
- `snk_pots.c` — `snk_pots_process_interface_socket()`

Each performs one `send()` and accepts a positive short result as success. None suppresses SIGPIPE.

### Complete-send utility

Add:

- `src/odas_ros/src/odas/include/odas/utils/socket.h`
- `src/odas_ros/src/odas/src/utils/socket.c`
- Register `socket.c` in ODAS `CMakeLists.txt`.

Implement a blocking `socket_send_all()` that:

1. Maintains a byte offset.
2. Calls `send()` with `MSG_NOSIGNAL`.
3. Advances on every positive return.
4. Retries `EINTR`.
5. Treats zero, `EPIPE`, connection reset, and other errors as failure.
6. Returns success only after all bytes are sent.

Use this helper from the SSS PCM, SST JSON, and SSL JSON sinks.

Preserve each caller’s current explicit fatal-error behavior. `MSG_NOSIGNAL` prevents unexpected signal termination, but a reported send failure still follows the deliberate ODAS error-exit path. Do not add in-process reconnect logic.

Change the dormant Python raw sender from `socket.send()` to `socket.sendall()`.

### PCM receiver root cause

File: `src/odas_ros/odas_ros/lib_odas_server_node.py`  
Class: `SssSocketServer`

A short `recv(4096)` result is discarded. The receiver has no persistent per-connection byte accumulator.

### PCM reconstruction

For each accepted SSS connection:

1. Create a fresh `bytearray`.
2. Calculate and validate:

   ```text
   frame_size = nbits / 8 × channel_count × frame_sample_count
   ```

   Current expected value: 4096 bytes.
3. Receive bounded chunks, for example `frame_size × 16`.
4. Append each chunk.
5. Extract and submit every complete 4096-byte frame in order.
6. Retain the incomplete tail.
7. After extraction, explicitly assert/check:

   ```text
   0 <= len(accumulator) < frame_size
   ```

8. On disconnect, diagnose and discard any incomplete tail.
9. Never preserve tail bytes across connections.

The accumulator may temporarily exceed one frame immediately after `recv()` because receiving multiple frames is valid. The strict bound applies after the extraction loop. Maximum transient storage is bounded by `recv_chunk_size + frame_size - 1`.

Do not clamp excess input or silently discard complete frames.

### JSON receiver

Leave `JsonSocketServer._json_buffer` and `_split_json()` unchanged. They already reconstruct fragmented JSON and extract multiple JSON objects from one receive.

Only add sender-side complete-send protection and ordinal validation of the parsed SST `timeStamp`.

### Tests

- One complete PCM frame.
- Every representative partial-frame split.
- Multiple frames in one receive.
- End of frame N plus beginning of N+1.
- Repeated small chunks.
- Partial tail at disconnect.
- New connection starts with an empty accumulator.
- Accumulator bound holds after every drain.
- Partial sends, `EINTR`, zero return, and `EPIPE`.
- `MSG_NOSIGNAL` is passed on every send.
- Peer disconnect produces the normal error path, not SIGPIPE termination.
- Existing fragmented/concatenated JSON parsing remains unchanged.

## 5. Fixed SST-to-SSS Channel Mapping

### Root cause

File: `src/odas_ros/odas_ros/lib_odas_server_node.py`  
Class: `SstSocketServer`  
Function: `_handle_data()`

The bridge removes `id == 0` entries and compresses the fixed ODAS track array. MobileNetV2 then incorrectly treats the compressed list index as the `/sss` channel.

### Representation

Do not change `OdasSst.msg` or `OdasSstArrayStamped.msg`.

Define and document the existing array position as the channel identity:

```text
/sst.sources[N] ↔ /sss channel N
```

`source.id` remains a track identifier:

- `id == 0`: empty fixed slot.
- `id > 0`: active track occupying that slot.

### Bridge implementation

In `SstSocketServer`:

1. Derive the expected slot count from `len(configuration['sst']['N_inactive'])`.
2. Require every decoded SST JSON object to contain exactly that many `src` entries.
3. Reject and diagnose the entire snapshot if the count differs.
4. Convert every entry, including `id == 0`.
5. Preserve JSON order exactly.
6. Never filter or sort by ID or activity.

Document the invariant in the relevant package README rather than modifying the ROS schema.

### MobileNetV2 implementation

In `MobileNetV2Classifier`:

- Continue accessing metadata by channel index.
- Reject snapshots whose array length differs from configured channel count.
- Define source identity as `(frame_id, channel_index, track_id)`.
- Never use `track_id` as a channel number.

### Required behavior

- Empty slots remain in the array.
- Non-contiguous sources retain their physical slots.
- Removing slot 1 does not move slot 2.
- Reusing a slot with a new track ID resets only that channel.
- AST and GRE positional assumptions become correct without algorithm changes.

### Tests

- `[0, A, 0, 0]`
- `[A, 0, B, 0]`
- `[0, 0, 0, 0]`
- Source disappearance.
- Non-contiguous sources.
- Track replacement in one slot.
- Incorrect JSON slot counts are rejected.
- `/sss` channel N is never associated with another slot’s metadata.

## 6. SST Multi-Candidate Allocation Fix

### Root cause

File: `src/odas_ros/src/odas/src/module/mod_sst.c`  
Area: dynamic “Add sources” loop.

Each qualifying potential searches only `obj->ids`. Newly selected slots remain zero there until the later update phase, allowing several candidates to reserve the same slot and overwrite its pending ID and Kalman state.

### Implementation

Reuse `idsAdded` as the current-frame reservation map.

A slot is available only when:

```c
obj->ids[iTrackMax] == 0 &&
obj->idsAdded[iTrackMax] == 0
```

For each qualifying potential:

1. Search slots in the existing order.
2. Reserve the first currently empty and unreserved slot.
3. Increment the track ID once.
4. Store it in `idsAdded` immediately.
5. Initialize that slot’s Kalman or particle state exactly once.
6. Initialize the existing probation fields.
7. Stop searching for that candidate.
8. If no free slot exists, skip the candidate without overwriting anything.

Do not change candidate ordering, thresholds, probation, deletion behavior, or tracking algorithms.

### Preserved thresholds

No changes to:

- `theta_new = 0.90`
- `N_prob = 5`
- `theta_prob = 0.80`
- `theta_inactive = 0.90`
- `N_inactive = 250`
- MobileNetV2 threshold `0.75`

SST values express spatial tracking confidence, not drone probability.

### Tests

- Two candidates plus two free slots.
- Four candidates plus four free slots.
- More candidates than free slots.
- Mixed occupied/free slots.
- A slot in `idsAdded` cannot be selected again.
- Each Kalman initialization receives its corresponding potential.
- Existing probation and threshold behavior remains unchanged.

## 7. Deterministic SSS/SST Ordinal Pairing

### Root cause

SSS and SST currently publish independently from separate TCP server threads. MobileNetV2 uses the latest SST message and timestamp proximity, so scheduling differences can associate audio with the wrong metadata or trigger unnecessary resets.

### Ordinal relationship (source-verified; validated at startup)

For the active configuration:

- Raw input produces one timestamped ODAS hop per 512 frames.
- SST propagates that hop timestamp and serializes it as JSON `timeStamp`.
- SSS explicitly requires the same track and spectral timestamp.
- ISTFT preserves it.
- Equal-rate SSS resampling produces one PCM frame per hop.
- The PCM sender strips the timestamp, but not the frame.

The bridge must validate this relationship at startup before enabling paired mode. Once validated, the relationship is:

```text
first reconstructed SSS frame (k=1) ↔ SST timeStamp baseline
second reconstructed SSS frame (k=2) ↔ SST timeStamp baseline + 1
...
SSS ordinal k ↔ SST timeStamp baseline + (k - 1)
```

### Shared bridge coordinator

Add an internal coordinator in `lib_odas_server_node.py`, owned by `OdasServerNode` and shared by `SstSocketServer` and `SssSocketServer`.

It must maintain:

- Active session generation.
- Connection state for both streams.
- Next expected SSS ordinal.
- Last accepted SST ordinal.
- Pending SSS frames keyed by ordinal.
- Pending SST snapshots keyed by ordinal.
- Pair counters, mismatch counters, and connection tokens.
- One synchronization mechanism covering all shared coordinator state.

The coordinator is shared by the SST and SSS server threads. Serialize every access to session generation, ordinals, pending maps, and pair counters under one lock, or route both streams through one thread-safe handoff queue. In particular, prevent both server threads from concurrently deciding that the same session is invalid and independently clearing or closing shared state.

### Session lifecycle

1. Each accepted SSS/SST socket receives a unique connection token.
2. Do not process a session until both current connections are established.
3. Hold reading at a connection barrier; kernel TCP buffering preserves early data.
4. When both are present:
   - Create a new internal session generation.
   - Clear prior pending data.
   - Set next SSS ordinal to 1.
   - Record `baseline` from the first received SST JSON `timeStamp`.
   - Pair SSS ordinal `k` with the SST message whose `timeStamp == baseline + (k - 1)`.
5. Increment the SSS ordinal exactly once for every fully reconstructed PCM frame.
6. Normalize each SST ordinal as `timeStamp - baseline + 1` before pairing.
7. Reject duplicate, decreasing, or skipped normalized SST ordinals diagnostically.
8. Pair only identical ordinals.
9. Never use nearest timestamp or arrival order as a fallback.
10. On either disconnect:
    - Invalidate the entire paired session.
    - Clear both pending maps and ordinal state.
    - Shut down the peer bridge socket so ODAS follows its normal send-error exit.
    - Ignore data from invalidated connection tokens.
11. A new session begins only after both new connections are present.

An absolute-start requirement would fail the entire session if ODAS starts its counter at 0 or if the first frame is lost during a connection-handshake race.

Because ODAS intentionally exits on socket send failure, reconnection means a new externally restarted ODAS process, not an in-process sink reconnect.

### Startup validation

Before trusting SST ordinals, confirm that the first SST object contains a JSON `timeStamp` field whose parsed value is an integer. Confirm on successive SST objects that the field increments by exactly 1. Also validate that the configured SSS rate and hop size produce exactly one reconstructed SSS frame per SST hop.

If either validation fails, disable paired mode with a clear error and fall back to the existing independent SST and SSS publication behavior. Do not guess an ordinal, coerce a non-integer value, or use timestamp proximity as a pairing fallback.

### Pair publication

After both components for ordinal N are available:

1. Validate the SST fixed-slot count.
2. Create one common ROS header stamp for the pair.
3. Publish `/sst` and `/sss` with exactly that stamp and frame ID.
4. Advance/remove ordinal N once.
5. Never publish the same ordinal twice.

Use a sample-time sequence for pair stamps:

```text
session_epoch + (ordinal - 1) × 512/44100
```

Anchor `session_epoch` when the first pair is completed. This keeps paired stamps identical and exposes missing ordinal gaps.

The pair was selected by ordinal; the shared stamp is only a downstream correlation token.

### Bounds and failures

Use bounded pending maps, maximum 64 entries per stream.

Introduce these named configurable limits:

- `session_invalidate_timeout`, default `0.5` seconds.
- `max_consecutive_pair_violations`, default `3`.

A pairing violation is any of the following:

- A duplicate ordinal appears,
- An ordinal gap is detected,
- A pending side exceeds 64 entries,
- Or an unmatched entry exceeds `session_invalidate_timeout`.

On the first tier, log the violation, drop or drain only the affected pairing window, increment the consecutive violation counter, mark classifier continuity as broken for the discarded window, and continue the session without shifting ordinal associations. A successfully completed subsequent pair resets the consecutive violation counter to zero.

On the second tier, when the counter reaches `max_consecutive_pair_violations`, invalidate the session, clear state, and shut down the peer socket to force a clean restart boundary. A TCP disconnect remains an immediate session invalidation and does not wait for the violation threshold.

When only SST or only SSS is configured, preserve its existing independent publication mode. Strict paired mode activates only when both socket outputs are enabled.

### Timestamp sanity checking

For each ordinal pair, compare SSS and SST bridge receive times.

- Existing `sst_timeout = 0.25` seconds becomes the warning threshold.
- An excessive difference increments a counter and emits a throttled warning.
- It does not change, reject, or remap the ordinal association.
- There is no nearest-timestamp fallback.

### Risks

- Strict mismatch handling can stop publication until ODAS restarts; this is preferable to silent misassociation.
- A hard single-strike kill under sustained CPU load can cause external-restart loops; the two-tier violation policy bounds misassociation risk while preserving availability.
- Pairing adds bounded latency while one side is delayed.
- The ordinal assumption depends on the current equal-rate, equal-hop configuration. Validate this at startup and disable paired mode with a clear error if the configuration no longer provides one SSS frame per SST hop.
- Exact PCM source timestamps remain unavailable without a wire-format change.

## 8. MobileNetV2 Coupling and Track Flapping

### Required dependency

MobileNetV2 must continue requiring:

- A separated `/sss` channel.
- SST metadata for the identical fixed slot.
- An unchanged nonzero track ID across a classification window.
- Valid current activity/direction before publishing an actionable direction.

Do not enable `always_classify` as a production workaround.

### Exact pair consumption

In `classifier_node.py`:

1. Cache fixed-slot `/sst` snapshots by their exact bridge-assigned header stamp.
2. Queue `/sss` frames briefly when their exact-stamp SST pair has not arrived yet.
3. Process only exact-stamp matches.
4. Do not use nearest timestamps or `latest_sst` to select metadata.
5. Drop and diagnose unmatched messages after `0.25` seconds, matching `sst_timeout`.
6. Keep queues bounded.
7. Detect missing pair-stamp intervals and reset channel continuity before later audio is appended.

Bound the classifier pending queue to 32 messages and the same 0.25-second lifetime so its buffering remains consistent with the bridge’s paired publication behavior.

### Reset rules

Reset a channel when:

- Its matched fixed slot has `id == 0`.
- Track ID changes.
- Coordinate frame changes.
- A paired ordinal is missing.
- Audio is malformed or stale.
- Audio timestamp continuity is broken.

Do not reset identity solely because activity for the same nonzero track temporarily crosses `min_activity`.

Correctly paired audio from the same nonzero track may continue accumulating during a short low-activity period, but publication remains prohibited until current activity is at least `min_activity`.

### Track-flapping regression

Explicitly test:

```text
slot N = track A
slot N = id 0
slot N = track B
```

At `id == 0`, clear:

- Streaming resampler state.
- Partial waveform window.
- Vote history.
- Stored identity.
- Cached actionable direction.

Track B must start with an empty state even when all events occur within the pairing timeout.

### Publication checks

Before publishing `/mobilenetv2/direction`, require:

- Positive MobileNetV2 vote under the unchanged 0.75 threshold.
- Exact paired SST metadata for the window end.
- Same slot and track ID as the accumulated audio.
- Current activity at least `min_activity`.
- Valid finite nonzero direction.
- Current track identity still matches the latest exact-slot SST state.

## 9. Implementation Order and Files

### Order

1. Add ALSA full-hop read logic and tests.
2. Add `socket_send_all()`, `MSG_NOSIGNAL`, and sender tests.
3. Add SSS PCM accumulator and reconstruction tests.
4. Fix SST multi-candidate reservations.
5. Preserve fixed SST slots in the ROS bridge.
6. Add shared SSS/SST session and ordinal coordinator.
7. Update MobileNetV2 to consume exact paired stamps.
8. Add track-flapping and end-to-end synthetic tests.
9. Update documentation of invariants and residual risks.

### Production files to modify

| File | Change |
|---|---|
| `src/odas_ros/src/odas/src/source/src_hops.c` | Correct ALSA precedence and accumulate complete interleaved hops |
| `src/odas_ros/src/odas/src/sink/snk_hops.c` | Use complete-send helper |
| `src/odas_ros/src/odas/src/sink/snk_tracks.c` | Use complete-send helper |
| `src/odas_ros/src/odas/src/sink/snk_pots.c` | Use complete-send helper |
| `src/odas_ros/src/odas/include/odas/utils/socket.h` | Declare complete-send helper |
| `src/odas_ros/src/odas/src/utils/socket.c` | Implement partial-send, EINTR, and SIGPIPE-safe behavior |
| `src/odas_ros/src/odas/CMakeLists.txt` | Compile utility and register C tests |
| `src/odas_ros/src/odas/src/module/mod_sst.c` | Reserve each free slot once per processing frame |
| `src/odas_ros/odas_ros/lib_odas_server_node.py` | PCM accumulator, fixed SST slots, paired sessions, ordinals, reconnect reset, and separate SST/SSS publisher depth 32 from SSL depth |
| `src/dracoviloc_odas/launch/audio_bringup.launch.py` | Increase `/sss` publisher history depth from 8 to 32 |
| `src/dracoviloc_mobilenetv2/mobilenetv2/classifier_node.py` | Exact-pair consumption and corrected state reset |
| `src/dracoviloc_mobilenetv2/README.md` | Document slot, pairing, and publication invariants |
| `src/dracoviloc_odas/README.md` | Document direct-soundcard bandpass bypass and device-selection risks |

Do not modify:

- ROS message schemas.
- `configuration.cfg` thresholds or hardware selection.
- ODAS SSL/SSS algorithms.
- MobileNetV2 model threshold.
- AST/GRE algorithms unless compatibility tests expose a concrete fixed-slot regression.

A ROS queue overflow after bridge pairing would produce an unmatched frame in the classifier. Publisher depth 32 covers approximately 0.37 seconds at 512 samples/44.1 kHz and is inexpensive protection against short scheduling stalls; keep SSL at its existing independent depth.

### Test files

- Add ODAS C tests for ALSA read accumulation, complete sends, and track-slot reservation.
- Add `src/odas_ros/test/test_socket_framing.py`.
- Add `src/odas_ros/test/test_pair_coordinator.py`.
- Extend `src/dracoviloc_mobilenetv2/test/test_node.py`.
- Update build/package test registration only as required.

## 10. Validation and Definition of Done

### Static/unit validation

All of the following must pass:

- ALSA full, short, repeated-short, zero, and error reads.
- No partial ALSA buffer is ever processed.
- Partial sends transmit all bytes.
- `EINTR` retries.
- `EPIPE` follows normal error handling without SIGPIPE termination.
- Arbitrary TCP chunking reconstructs byte-identical 4096-byte frames.
- Accumulator tail is always smaller than one frame after draining.
- SST fixed slots remain fixed and non-contiguous.
- Multiple SST candidates obtain unique slots.
- SSS ordinals 1, 2, 3 pair only with SST timestamps `baseline`, `baseline + 1`, and `baseline + 2`.
- Artificial thread delays do not alter associations.
- Duplicate, skipped, decreasing, timed-out, or overflowing pairing windows are logged and drained without shifting; three consecutive violations invalidate the session.
- Disconnect/restart resets SSS ordinal state to 1 and establishes a new SST baseline.
- `A → 0 → B` cannot leak state from A to B.
- SST thresholds and MobileNetV2 threshold remain unchanged.

### Synthetic integration validation

Without a microphone:

1. Feed controlled TCP PCM chunks and SST JSON messages.
2. Vary chunk sizes and arrival scheduling independently.
3. Verify paired `/sss` and `/sst` headers are identical per ordinal.
4. Verify no nearest-timestamp association occurs.
5. Verify delayed but matching ordinals pair correctly.
6. Verify cross-session data is never paired.
7. Publish synthetic fixed-slot ROS messages into the classifier callback tests.
8. Confirm only the correct channel and track can produce a direction.

### Residual risks

Document, but do not fix:

- ALSA XRUN errors can still stop capture.
- Current direct soundcard path bypasses the feeder’s 180–3600 Hz Butterworth bandpass.
- Device selection remains configuration-specific at `card=2`, `device=0`, producing `hw:2,0`.
- ODAS deliberately exits after a socket send failure; restart remains external.
- Ordinal pairing depends on equal one-frame-per-hop SST/SSS configuration.
- SSS PCM still lacks an explicit wire-level ODAS timestamp.

### Completion criteria

The corrective patch is complete when:

- Only complete fresh ALSA hops enter processing.
- TCP sends cannot silently truncate.
- TCP receives cannot lose or misalign PCM frame boundaries.
- SIGPIPE cannot bypass the documented error path.
- `/sst.sources[N]` always describes `/sss` channel N.
- SST candidates never share a newly reserved slot.
- SSS and SST are paired strictly by same-session ordinal.
- First-pair ordinals and baseline are logged at every session start.
- Timestamps are diagnostic only and never select another pair.
- Disconnects clear all session, accumulator, and classifier association state.
- Empty-slot track flapping always resets prior classification state.
- All claimed tests have actually run and passed, with unavailable tests reported explicitly.
