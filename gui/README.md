# DracoViLoc Control Center

Local PySide6/Qt Quick operations console for DracoViLoc. It supervises the
host ROS launch and Isaac ROS container, observes ROS health, controls the
existing home/tracking services, and browses recordings. It never starts arm
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

## Safety

- Start Run starts processes only. It does not home or arm tracking.
- Home and Enable Tracking use 1.5-second hold gestures plus confirmations.
- Stop Motion requests `/demo/tracking=false` and keeps sensing/recording alive.
- Stop All requests a motion stop before shutting down managed processes.
- Always keep the FAIRINO physical emergency stop within reach.

