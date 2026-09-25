# DracoViLoc — Laptop Port & Audio Pipeline Corrections (2026-09-24)

This document summarizes everything that was added and fixed while bringing
DracoViLoc up on the x86 laptop (Ubuntu 22.04, RTX 5050), with special focus
on the two audio problems that motivated the work: **the `/sss` stream
recording nothing**, and **recorded/classified sound being far too quiet**.

The original Jetson workflow remains documented in `README.md`; the
copy-paste launch workflow for this machine is in `INSTRUCTION.md`.

---

## 1. What was done (overview)

| Area | Result |
|---|---|
| Environment | Full ROS 2 Humble + CUDA 12.8 + TensorRT 11 x86 setup installed |
| Models | All TensorRT engines rebuilt for the RTX 5050 (committed ones were Jetson binaries); Git LFS weights pulled |
| Audio pipeline | 5 corrective fixes from `reportv2.md` implemented (C + Python), 52 tests passing |
| New feature | Per-channel `/sss` WAV recorder (`recordings/<date_time>/channel_N.wav`) |
| Loudness | +24 dB digital gain on `/sss` (bridge, default ON) + optional trim gain in the recorder |
| Bug fix | AST/GRE launch-argument collision in `arm_audio_demo.launch.py` |
| Verification | Live drone-audio tests: AST 82–90% [DRONE], GRE decision=DRONE, recordings loud and valid |

---

## 2. How the sound was FIXED (`/sss` recording nothing)

### Symptoms

`/sss` always appeared to contain silence: recorded WAVs were all zeros, on
both the old laptop and this one. The report (`reportv2.md`, kept at the repo
root) identified five real bugs in the capture → TCP → ROS bridge chain; all
were verified to exist in the code and are now fixed.

### The five fixes

**Fix 1 — ALSA capture read bug (root cause of silent/corrupt hops)**
`src/odas_ros/src/odas/src/source/src_hops.c`,
`src_hops_process_interface_soundcard()`

The old code did `if (err = snd_pcm_readi(...) > 0)` — an operator-precedence
bug that also treated a *short* read as a complete hop, leaving the rest of
the capture buffer stale (often initialized silence). Replaced with a
full-hop loop using `snd_pcm_sframes_t`: read until exactly `hopSize` (512)
interleaved frames are captured, offset the destination per accumulated
frames, fail on zero/negative returns. No XRUN recovery added, pipeline order
unchanged.

**Fix 2 — TCP senders could truncate frames**
New files: `src/odas_ros/src/odas/include/odas/utils/socket.h`,
`src/odas_ros/src/odas/src/utils/socket.c` (registered in
`src/odas_ros/src/odas/CMakeLists.txt`)

The three TCP sinks did a single `send()` and accepted a positive short
result as success, so a 4096-byte PCM frame could be silently truncated.
New `socket_send_all()`: byte-offset loop, `MSG_NOSIGNAL` (prevents SIGPIPE
kills), `EINTR` retry, fail on zero/`EPIPE`/reset, success only when all
bytes are sent. Used by `src/odas_ros/src/odas/src/sink/snk_hops.c`,
`snk_tracks.c`, `snk_pots.c`, each keeping its existing fatal-error path.

**Fix 3 — TCP receiver discarded short reads**
`src/odas_ros/odas_ros/lib_odas_server_node.py`, class `SssSocketServer`

The bridge did `recv(frame_size)` and **threw away any short read**
(`len(data) != recv_size: continue`), losing and misaligning PCM frames.
Now a per-connection `PcmFrameExtractor` accumulator (new ROS-free class,
unit-tested) receives bounded chunks, reconstructs every complete 4096-byte
frame in order, keeps only a sub-frame tail (`0 <= tail < frame_size`), and
discards/diagnoses the tail at disconnect.

**Fix 4 — `/sst` fixed track slots (channel identity)**
`src/odas_ros/odas_ros/lib_odas_server_node.py`, class `SstSocketServer`

The bridge removed `id == 0` entries and compressed the array, so
`/sst.sources[N]` no longer corresponded to `/sss` channel N (classifiers
were reading the wrong channel's metadata). Now the expected slot count is
derived from `len(configuration['sst']['N_inactive'])`, every snapshot with
the wrong count is rejected, and **all entries including `id == 0` are
published in order**. Invariant: `/sst.sources[N]` ↔ `/sss` channel N.

**Fix 5 — SST slot double-reservation + deterministic SSS/SST pairing**
`src/odas_ros/src/odas/src/module/mod_sst.c` and
`src/odas_ros/odas_ros/lib_odas_server_node.py`

- `mod_sst.c`: in the dynamic "Add sources" loop a slot is now available only
  when `ids[i] == 0 && idsAdded[i] == 0`, so multiple candidates can no
  longer reserve the same newly-free slot.
- `lib_odas_server_node.py`: new `PairCoordinator` owned by
  `OdasServerNode`, shared by both socket-server threads under one lock.
  `/sss` and `/sst` are published as pairs matched strictly by ODAS-hop
  ordinal (SSS frame k ↔ SST `timeStamp == baseline + (k-1)`) with identical
  header stamps, connection-token session lifecycle, bounded pending maps,
  two-tier violation policy, and immediate session invalidation on
  disconnect. Startup validation falls back to independent publication if
  the ordinal relationship doesn't hold. No nearest-timestamp matching
  anywhere.

### Tests added

- `src/odas_ros/src/odas/test/test_src_hops.c`,
  `src/odas_ros/src/odas/test/test_socket.c` (C, CTest-registered; link-time
  `--wrap` mocking of `snd_pcm_readi`/`send`)
- `src/odas_ros/test/test_socket_framing.py`,
  `src/odas_ros/test/test_pair_coordinator.py`,
  `src/odas_ros/test/stub_ros.py` (52 Python tests total)

Run: `colcon test --packages-select odas_ros` and
`cd src/odas_ros && python3 -m pytest test/ -v`.

### The other half of the story: it was also *genuinely quiet*

After the fixes, silence recordings were still exactly zero — correctly.
Measurements showed why: laptop/phone speakers deliver only about **-30 dBFS
at the array** (ALSA capture already at max; the UMA16v2 exposes no analog
boost on Linux). A quiet room yields legitimately zero separated audio, and
soft sources vanish below ODAS's tracking thresholds. A loud, close source
(e.g. a phone playing drone audio next to the array) produces healthy audio.
So: real bugs (now fixed) + weak acoustic staging (addressed below).

---

## 3. How the sound was made LOUD

### Where the gain lives

**`/sss` bridge gain — enabled by default (+24 dB)**
`src/odas_ros/odas_ros/lib_odas_server_node.py`

- New parameter `sss_gain_db` on `odas_server_node`, **default 24.0**
  (≈15.85× amplitude), so every consumer gets loud audio out of the box:
  AST, GRE, and both recorders.
- Applied in both publish paths (`_send_sss` and `publish_pair_frame`) via a
  pure, unit-tested function `apply_sss_gain()`: samples are converted to
  float, multiplied, and **clamped to int16 range** — an over-hot source
  distorts but never corrupts frame alignment. `gain == 1.0` is an identity
  pass-through (zero cost when disabled).
- Logged at startup: `/sss gain: 24.0 dB (15.85x)`. Override with
  `-p sss_gain_db:=0.0` to publish the raw ODAS level.

**Recorder trim gain (optional, default 0)**
`src/dracoviloc_recording/src/sss_channel_recorder.cpp`

- `gain_db` parameter (same float math + saturation) applied per channel
  while deinterleaving, before writing. Exposed as `gain_db` in
  `src/dracoviloc_recording/launch/sss_channels.launch.py` and as
  `sss_channels_gain_db` in `arm_audio_demo.launch.py`.
- Keep it at 0.0 in normal use — `/sss` is already boosted at the bridge;
  adding both gains double-boosts (+48 dB) and clips.

### Measured results (live, drone audio from a phone near the array)

| Stage | Level before | Level after |
|---|---|---|
| `/sss` peak (published) | ~369 (-39 dBFS) | **4041 (-18.2 dBFS), 0 clipped samples / 7M** |
| AST confidence | 56–86 % [DRONE] | **82.7–89.8 % [DRONE]** |
| GRE | decision=DRONE | decision=DRONE, confidence up to 0.95 |
| Recorded WAV | peak 369 (-39 dBFS) | **peak 5816 (-15 dBFS), loud and clear** |

---

## 4. New feature: per-channel `/sss` recorder

New files:
- `src/dracoviloc_recording/src/sss_channel_recorder.cpp` — C++ node
  subscribing to `/sss`; deinterleaves the stream and writes one mono
  16-bit WAV per channel (`channel_0.wav` …), 44-byte placeholder RIFF
  headers finalized on clean shutdown, plus `metadata.json`
  (channels/rate/gain/frames/samples/state).
- `src/dracoviloc_recording/launch/sss_channels.launch.py` — standalone
  launch (`audio_topic`, `output_root`, `gain_db` args).
- Edited `src/dracoviloc_recording/CMakeLists.txt` — new target +
  `install(DIRECTORY launch …)`.

Usage: `sss_channels_recording:=true` in the main launch, or the standalone
launch from a second terminal. Output: `recordings/<DD_MM_YYYY_HH_MM_SS>/`
(repo root, gitignored).

## 5. Launch bug fix (existed on the Jetson too)

`src/dracoviloc_bringup/launch/arm_audio_demo.launch.py`

AST and GRE are included as sibling launch descriptions, and ROS 2 shares
the launch context across them — so GRE inherited AST's `venv_python`,
`engine_path`, etc., and started with the AST engine and wrong venv. Fixed by
passing explicit `venv_python` / `engine_path` / `meta_path` / `model_dir`
to both includes. This file also gained the `sss_channels_recording`,
`sss_channels_root`, and `sss_channels_gain_db` arguments.

---

## 5a. RViz phantom arrows (side effect of Fix 4)

After the fixed-slot `/sst` change, RViz showed several stray red arrows
pointing nowhere in silence. Cause: `odas_visualization_node.py` drew one
PoseArray arrow per `/sst` entry, including empty slots (`id == 0`, zero
direction). Fix in `src/odas_ros/scripts/odas_visualization_node.py`
(`_sst_cb`): skip `id == 0` slots. Verified: `/sst_poses` is empty in
silence, exactly one arrow appears while a source is tracked.

## 6. Full list of files edited/added

**Audio corrections (reportv2.md):**
- `src/odas_ros/src/odas/src/source/src_hops.c` *(edited — Fix 1)*
- `src/odas_ros/src/odas/src/sink/snk_hops.c`, `snk_tracks.c`, `snk_pots.c` *(edited — Fix 2)*
- `src/odas_ros/src/odas/include/odas/utils/socket.h` *(new — Fix 2)*
- `src/odas_ros/src/odas/src/utils/socket.c` *(new — Fix 2)*
- `src/odas_ros/src/odas/CMakeLists.txt` *(edited — register socket.c + C tests)*
- `src/odas_ros/src/odas/src/module/mod_sst.c` *(edited — Fix 5a)*
- `src/odas_ros/odas_ros/lib_odas_server_node.py` *(edited — Fixes 3, 4, 5b + `sss_gain_db`)*
- `src/odas_ros/scripts/odas_visualization_node.py` *(edited — RViz phantom arrows, see §5a)*
- `src/odas_ros/src/odas/test/test_src_hops.c`, `test_socket.c` *(new)*
- `src/odas_ros/test/test_socket_framing.py`, `test_pair_coordinator.py`, `stub_ros.py` *(new)*

**Loudness:**
- `src/odas_ros/odas_ros/lib_odas_server_node.py` *(bridge +24 dB default, see above)*

**Per-channel recorder:**
- `src/dracoviloc_recording/src/sss_channel_recorder.cpp` *(new)*
- `src/dracoviloc_recording/launch/sss_channels.launch.py` *(new)*
- `src/dracoviloc_recording/CMakeLists.txt` *(edited)*

**Launch integration:**
- `src/dracoviloc_bringup/launch/arm_audio_demo.launch.py` *(AST/GRE arg fix + recorder args)*

**Docs & repo:**
- `INSTRUCTION.md` *(new — x86 laptop workflow, updated with corrections/gain)*
- `.gitignore` *(added `/recordings/`)*
- `reportv2.md` *(pre-existing corrective plan, kept at repo root)*

**Environment (not in git):** ROS 2 Humble, CUDA 12.8, TensorRT 11.3, CUDA
rebuilt AST/GRE engines (`models/ast/drone_ast.engine`,
`models/gre/model_logmel.engine` + copies under
`src/dracoviloc_audio_fusion/models/`), `gre_env`/`trt_env` venvs,
`~/DracoViLoc` → `~/Desktop/DracoViLoc` symlink (required by AST/GRE launch
defaults).

## 7. Known residual limitations

- ALSA XRUNs can still stop capture (no recovery by design).
- The direct soundcard path bypasses the feeder's 180–3600 Hz bandpass.
- Device selection is hardcoded at `hw:2,0` in
  `src/odas_ros/config/configuration.cfg`.
- ODAS intentionally exits on socket send failure; restart is external
  (relaunch the stack).
- Very loud sources at +24 dB bridge gain can clip in the published `/sss`;
  lower `sss_gain_db` if that is ever a problem.
