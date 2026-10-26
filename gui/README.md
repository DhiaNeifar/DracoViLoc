# DracoViLoc Control Center

PySide6/Qt Quick operations console for DracoViLoc. It can supervise the host
ROS launch and Isaac ROS container either locally or on a Jetson over an
SSH-tunneled agent, observes ROS health, controls the existing home/tracking
services, and browses recordings. It never starts arm
motion automatically and is not a replacement for the physical emergency
stop.

## Install and run

```bash
cd ~/DracoViLoc
./scripts/setup_gui.sh
./scripts/run_gui.sh
```

The setup creates `gui_env` with system site packages enabled so the GUI can
use the ROS Humble `rclpy` installation, and installs the desktop entry for
the current user. The desktop file is intentionally specific to this
workstation. Continue to use `scripts/stop_dracoviloc.sh` as the command-line
recovery path.

## Processes and logging

Local runs start every subsystem as a transient systemd user unit
(`dracoviloc-host.service`, `dracoviloc-yolo.service`, ...) over D-Bus.
systemd supervises each one: it keeps running if the window closes or
crashes, the GUI reattaches to it on restart, and stopping sends Ctrl+C
(SIGINT) with a SIGKILL fallback after the stop timeout. Inspect them with
`systemctl --user list-units 'dracoviloc-*'`.

Each session writes to `runs/logs/<YYYY-MM-DD_HH-MM-SS>/`:

| File | Contents |
|---|---|
| `session.json` | machine, user, git commit/branch/dirty, GUI version, start and end time |
| `events.jsonl` | every operator action (button, setting change with old/new value, navigation, dialog), state change, message, process start/stop/exit, warning and error |
| `processes/<name>_<time>.log` | the full terminal output of each process, line by line with timestamps (ANSI colors preserved) |
| `processes/<name>_<time>.status.json` | command, PIDs, start/stop/end time, exit code or signal |
| `runs/run_<time>.json` | the profile and exact commands of each run |

Actions, state changes and errors are written to disk (fsync) as they happen.
The Logs page shows the events and one live terminal per process; logs stay
on the machine and are not synced to Drive.

Whenever the camera (YOLO) is enabled, the raw `/image` stream is always
recorded to `runs/video/<time>/camera.mp4`; this cannot be switched off, and
overlay images are never recorded. Preflight warns when this machine cannot
upload recordings to Google Drive (see `docs/DRIVE_SYNC.md`).

## Setup & builds

The **Setup & builds** page checks each component of this machine and runs
the script that fixes it, with its live terminal output (also in Logs):

| Card | Status check | Action |
|---|---|---|
| Repository | branch, commit, local changes, commits behind the last fetch | `git pull --ff-only` (disabled with local changes) |
| FAIRINO SDK | `libfairino.so` present and built for this CPU (read from its ELF header) | `scripts/build_fairino_sdk.sh` |
| DracoViLoc workspace | sources newer than each package's last successful colcon build | `colcon build --symlink-install --base-paths src` |
| Project Docker image | `dracoviloc/isaac_ros:<arch>-<hash>` exists; the hash covers `isaac_ros/docker/` | `docker build` on the Isaac ROS dev image for this CPU |
| Isaac ROS workspace sources | `deploy_isaac_ros.sh --dry-run` lists files that differ | `scripts/deploy_isaac_ros.sh` |
| Isaac ROS build | YOLO package sources newer than their container build | `build_dracoviloc_yolo.sh` in a one-off project container |

**Bring this machine up to date** walks these steps in order, re-checking
each one just before it would run, and stops at the first failure. After a
`git pull` on either machine it is the only button needed. Jobs and runs
are mutually exclusive.

The project image bakes in every runtime dependency, so container builds
never install packages. **Save image** writes a compressed copy to
`images/` (git-ignored); copy it to the same folder on another machine with
the same CPU type and **Load** it there. YOLO runs and builds use the
project image when it exists, and run as the host user so files in the
Isaac workspace keep your ownership.

## Remote Jetson control

1. Configure a key-based SSH host alias named `jetson` (or enter another alias
   in Run setup). Verify it once with `ssh jetson`; strict host-key checking is
   used and password prompts are not handled inside the GUI.
2. In Run setup, choose `remote`, confirm the three Jetson paths, then select
   **Deploy / update agent**. This installs and starts a loopback-only systemd
   user service through SSH.
3. Select **Connect**, then **Take control**. Only one GUI can own control;
   additional clients remain read-only.
4. Run preflight and start normally. RViz is intentionally disabled in remote
   mode. The dashboard receives `/yolov8_processed_image` as a 10 FPS JPEG
   preview. Completed recordings remain on the Jetson until **Download** is
   selected; downloads are stored under `remote_downloads/`.

The agent never opens a LAN port: it listens on Jetson loopback and is reached
only through the SSH tunnel. If the laptop disconnects, the Jetson run and any
active tracking continue. The GUI reconnects automatically, but another client
cannot take control until the disconnected lease expires. To keep the user
services running across logout/reboot, follow the deployment warning and run
`sudo loginctl enable-linger $USER` once on the Jetson.

## Safety

- Start Run starts processes only. It does not home or arm tracking.
- Home and Enable Tracking use 1.5-second hold gestures plus confirmations.
- Stop Motion requests `/demo/tracking=false` and keeps sensing/recording alive.
- Stop All requests a motion stop before shutting down managed processes.
- Always keep the FAIRINO physical emergency stop within reach.

## Two-cobot operation

Choose the **Dual Cobot EKF** profile or enable **Two cobots** in Run setup.
Configure distinct left and right controller addresses, then enter the measured
right-cobot base offset relative to the first cobot. The GUI passes the existing
`dual_arm_controller`/`dual_arm_tracking_controller` configuration through both
local and Jetson execution modes and requires all 12 joint states before both
cobots are considered ready. Preflight checks both physical controllers.

`first_arm_lead` is the calmer default: the sensor-side primary cobot aligns
first and the follower moves after the configured angular tolerance is met.
`simultaneous` moves both cobots immediately. Homing, tracking enable, motion
stop, and Stop All apply to both cobots as one coordinated controller group.

