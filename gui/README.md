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

