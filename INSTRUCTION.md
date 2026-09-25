# DracoViLoc — Launch Instructions (this laptop)

How to build and run DracoViLoc from scratch on this machine (x86_64 Ubuntu 22.04,
RTX 5050). The original Jetson instructions live in `README.md`; this file is the
x86 laptop workflow that was installed and verified on 2026-09-24.

## What runs where

- **Host (this laptop):** DracoViLoc stack, RViz, ODAS + UMA16v2 audio, AST/GRE
  classification, mock FAIRINO arm (or real arm later).
- **Docker container (not installed yet):** Isaac ROS YOLO — added later, see
  "YOLO (later phase)" below.
- The EKF stays disabled (`fusion_enabled:=false`) until sensors/frames are
  validated, same policy as the Jetson setup.

## One-time setup (already done on this machine)

Skip this section unless you are reinstalling from scratch.

1. System packages:
   ```bash
   sudo apt update
   sudo apt install -y python3.10-venv libportaudio2 portaudio19-dev \
     build-essential cmake git curl git-lfs
   ```
2. ROS 2 Humble (Ubuntu 22.04):
   ```bash
   sudo add-apt-repository universe -y
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
     -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
     | sudo tee /etc/apt/sources.list.d/ros2.list
   sudo apt update
   sudo apt install -y ros-humble-desktop ros-dev-tools ros-humble-ros2controlcli
   ```
3. CUDA 12.8 + TensorRT 11 (PyPI has no Blackwell wheels — use NVIDIA apt repo):
   ```bash
   wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
   sudo dpkg -i cuda-keyring_1.1-1_all.deb
   sudo apt update
   sudo apt install -y cuda-toolkit-12-8 tensorrt python3-libnvinfer
   ```
   Verify: `/usr/local/cuda-12.8/bin/nvcc --version`, `trtexec --version`,
   `python3 -c "import tensorrt; print(tensorrt.__version__)"`
4. FAIRINO SDK (x86_64) and workspace dependencies:
   ```bash
   cd ~/Desktop/DracoViLoc
   ./scripts/build_fairino_sdk.sh
   sudo rosdep init   # once per machine; skip if already initialized
   rosdep update
   source /opt/ros/humble/setup.bash
   rosdep install --from-paths src --ignore-src -r -y
   sudo apt install -y libfftw3-dev libasound2-dev libconfig-dev libpulse-dev \
     python3-libconf ros-humble-moveit-planners-ompl
   ```
5. Model files are Git LFS — pull the real weights/ONNX:
   ```bash
   git lfs install
   git lfs pull
   ```
6. Python environments:
   ```bash
   cd ~/Desktop/DracoViLoc
   python3 -m venv --system-site-packages gre_env
   python3 -m venv --system-site-packages trt_env
   gre_env/bin/pip install pyyaml soundfile sounddevice pandas 'numpy==1.24.4'
   trt_env/bin/pip install soundfile sounddevice 'transformers==4.46.3' 'numpy==1.24.4'
   # pycuda needs nvcc:
   export PATH=/usr/local/cuda-12.8/bin:$PATH CUDA_HOME=/usr/local/cuda-12.8
   gre_env/bin/pip install pycuda
   trt_env/bin/pip install pycuda
   ```
   Verify each: `gre_env/bin/python -c "import pycuda.autoinit"` (numpy must be
   ≥1.22 — the pin above is required, system numpy 1.21 breaks pycuda).
7. Rebuild the TensorRT engines for THIS GPU (committed engines are
   Jetson binaries and will not load here):
   ```bash
   cd ~/Desktop/DracoViLoc
   trtexec --onnx=models/gre/model_logmel.onnx --saveEngine=models/gre/model_logmel.engine
   trtexec --onnx=models/ast/drone_ast.onnx   --saveEngine=models/ast/drone_ast.engine
   cp models/gre/model_logmel.engine src/dracoviloc_audio_fusion/models/model_logmel.engine
   cp models/ast/drone_ast.engine    src/dracoviloc_audio_fusion/models/drone_ast.engine
   ```
   Note: TensorRT 11 removed `--fp16` (strongly-typed networks are the default).
8. Symlink required by the AST/GRE launch defaults (they hardcode
   `$HOME/DracoViLoc`):
   ```bash
   ln -sfn ~/Desktop/DracoViLoc ~/DracoViLoc
   ```
9. Build the workspace:
   ```bash
   cd ~/Desktop/DracoViLoc
   source /opt/ros/humble/setup.bash
   colcon build --symlink-install --base-paths src
   ```
   `--base-paths src` is required: without it colcon scans `gre_env/` and
   `trt_env/` and fails on duplicate test packages.

## Every-day launch

### 0. Terminal setup (do this in every new terminal)

```bash
cd ~/Desktop/DracoViLoc
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Do **not** source the venvs manually (the launch files pick the right venv
python per node), and do not source any Isaac ROS workspace here.

### 1. Plug in the UMA16v2 and confirm it is card 2

```bash
arecord -l        # expect: card 2: UMA16v2 [UMA16v2] ... USB Audio
```

If it shows a different card number, see Troubleshooting before launching.

### 2. Launch the full stack (audio + AST + GRE + RViz, mock arm)

```bash
ros2 launch dracoviloc_bringup arm_audio_demo.launch.py \
  audio_enabled:=true ast_enabled:=true gre_enabled:=true \
  yolo_enabled:=false fusion_enabled:=false \
  tracking_mode:=off use_rviz:=true
```

Useful variants:

- Visualization only, no audio:
  `audio_enabled:=false ast_enabled:=false gre_enabled:=false`
- Record the run (video/audio require the sources to be enabled):
  add `recording_enabled:=true recording_root:=/home/dhia/Desktop/DracoViLoc/runs`
- Arm tracking once validated:
  `tracking_mode:=direct_ast` (or `direct_gre` / `direct_either`)

Stop everything cleanly with **Ctrl+C** (finalizes WAV/MP4 when recording).

### 3. Per-channel audio recording (optional)

The `/sss` stream is published with a **+24 dB digital gain by default**
(`sss_gain_db` parameter on `odas_server_node`) because the UMA16v2 has no
analog boost on Linux. Classifiers (AST/GRE) and recorders therefore receive
loud audio out of the box. To publish the raw ODAS separation level instead,
override with `-p sss_gain_db:=0.0`.

Record `/sss` into one mono WAV per channel, independent of the video/SST
recorder. Output layout:

```text
recordings/24_09_2026_15_04_11/
├── channel_0.wav ... channel_3.wav   # mono, 16-bit PCM, 44.1 kHz
└── metadata.json
```

Standalone (while the main stack is running, from a second sourced terminal):

```bash
ros2 launch dracoviloc_recording sss_channels.launch.py
```

Or together with the main launch:

```bash
ros2 launch dracoviloc_bringup arm_audio_demo.launch.py \
  ... sss_channels_recording:=true
```

The recorder's own `sss_channels_gain_db` defaults to 0.0 — keep it there,
since `/sss` is already boosted at the bridge; use it only for extra trim.

Stop with Ctrl+C so the WAV headers and metadata are finalized. Play back
with `aplay recordings/<folder>/channel_N.wav`.

### 3c. Audio pipeline corrections (applied 2026-09-24, from reportv2.md)

The ODAS capture/transport/bridge layer was corrected for five bugs that
corrupted or silently dropped separated audio:

- ALSA capture now accumulates exactly one complete 512-frame hop per read
  (the old code had an operator-precedence bug and treated short reads as
  complete hops).
- All TCP sinks send complete buffers with `MSG_NOSIGNAL` (single short
  `send()` calls could truncate PCM frames).
- The bridge reconstructs 4096-byte PCM frames from arbitrary TCP chunking
  with a per-connection accumulator (short `recv`s were previously discarded).
- `/sst` keeps fixed track slots: `sources[N]` always describes `/sss`
  channel `N`; `id == 0` means the slot is empty (previously empty slots were
  removed and positions shifted).
- `/sss` and `/sst` are published as pairs matched by ODAS-hop ordinal with
  identical header stamps (previously two independent threads, matched by
  "latest message").

Run the test suites after touching `odas_ros`:

```bash
cd ~/Desktop/DracoViLoc
source /opt/ros/humble/setup.bash
colcon test --packages-select odas_ros && colcon test-result
cd src/odas_ros && python3 -m pytest test/ -v   # bridge unit tests (52 tests)
```

### 4. Verify it is alive

```bash
ros2 topic hz /sss                 # separated audio, ~86 Hz
ros2 topic hz /sst                 # sound source tracks, ~86 Hz
ros2 topic hz /ssl_pcl2            # RViz point cloud
ros2 topic echo /sst --once        # always 4 entries; id: 0 = empty slot
ros2 control list_controllers      # joint_state_broadcaster + arm_controller active
```

Classifier output prints in the launch terminal: `AST confidence ...` and
`GRE confidence=... decision=DRONE ...` whenever a window is classified.

## YOLO (later phase — not installed yet)

1. Install Docker + NVIDIA Container Toolkit.
2. Follow `isaac_ros/README.md` and `scripts/build_isaac_ros.sh` /
   `deploy_isaac_ros.sh` (the dev container also works on x86_64).
3. Export YOLO ONNX from `models/yolo/drone_yolo11n_20260825_best.pt` and build
   a new `.plan` **inside the container** for the RTX 5050.
4. Run YOLO in the container, then the host launch with `yolo_enabled:=true
   tracking_mode:=direct_yolo`. Never source the container workspace on the host.

## Real FAIRINO arm (later phase — not connected yet)

The SDK is already built (`src/fairino_hardware/libfairino/`). When the arm is
available: set `hardware_mode:=real` and `robot_ip:=<arm-ip>` in the launch,
validate `/joint_states` feedback, then enable a tracking mode.

## Troubleshooting

- **`Cannot open audio device hw:2,0`** — the UMA16 is not card 2. Find it with
  `arecord -l` and update `hw:X,0` in `src/odas_ros/config/configuration.cfg`.
  For a permanent fix, add a udev rule pinning the UMA16 to a fixed ALSA name.
- **`No module named 'libconf'`** — `sudo apt install python3-libconf`.
- **pycuda `numpy._DTypeMeta object is not subscriptable`** — numpy too old in
  the venv; `pip install 'numpy==1.24.4'` in that venv.
- **AST/GRE never start, or GRE loads the AST engine** — keep the
  `~/DracoViLoc` symlink (step 8) and keep the launch fix in
  `arm_audio_demo.launch.py` (explicit `venv_python`/`engine_path` per include).
- **colcon fails with duplicate `my-test-package`** — you built without
  `--base-paths src`; rebuild with it.
- **TensorRT engine load failure** — the engine was built for another machine.
  Rebuild with `trtexec` from the ONNX in `models/` (step 7).
- **MoveIt dies at startup** — `sudo apt install ros-humble-moveit-planners-ompl`.
- **`Address already in use` on ports 9000/9001/10000** — an old stack is
  still running. Kill it (`pkill -9 -f odas_core_node` plus the launch
  process) and relaunch.
- **`/sss` records all zeros** — usually normal, not a bug: separated audio
  is per-track, so only channels with an active `/sst` track (id != 0) carry
  sound; silence is exactly zero. The source must also be loud enough — phone
  speakers at arm's length reach only ~-30 dBFS at the mics. Check
  `ros2 topic echo /sst` for a non-zero id while the sound plays, and use the
  `uma16_feeder --no-publish` level check (main stack stopped) to measure
  `-dBFS` at the array.
- **Recorded audio distorts / clips** — the bridge already applies +24 dB by
  default; make sure `sss_channels_gain_db` is 0.0 (double gain clips), or
  lower `sss_gain_db` on the odas server node.
- **Recording folder has WAVs but no `metadata.json`** — the stack was killed
  without a clean Ctrl+C; the WAV headers were still finalized, only the
  counters/metadata were lost.
