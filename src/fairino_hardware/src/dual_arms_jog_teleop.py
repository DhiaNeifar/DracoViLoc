#!/usr/bin/env python3
"""
Dual-Arm Direct JOG Control via Keyboard
Uses StartJOG/StopJOG commands directly for both arms without MoveIt planning.

Control Mapping:
  Translation (Cartesian):
    a/d     → X-axis (MIRRORED)
    w/s     → Y-axis (SAME)
    q/e     → Z-axis (SAME)
  
  Rotation (Cartesian):
    j/l     → Roll/A (SAME)
    i/k     → Pitch/B (SAME)
    u/o     → Yaw/C (MIRRORED)
  
  Control:
    r       → Reset errors on both robots
    h       → Home both robots
    ESC     → Quit

Author: GitHub Copilot
Date: March 30, 2026
"""

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from fairino_msgs.srv import RemoteCmdInterface
from sensor_msgs.msg import Joy
import sys
import select
import termios
import tty
import time
import subprocess
import os
import signal


class DualArmsJogTeleop(Node):
    def __init__(self):
        super().__init__('dual_arms_jog_teleop')

        self.declare_parameter(
            'start_hardware_servers',
            'auto',
            ParameterDescriptor(dynamic_typing=True),
        )
        self.declare_parameter('left_robot_ip', '192.168.58.2')
        self.declare_parameter('right_robot_ip', '192.168.58.3')
        self.declare_parameter('use_keyboard', True)
        self.declare_parameter('use_joy', True)
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('joy_timeout_sec', 0.25)
        self.declare_parameter('threshold', 0.1)
        
        # Server processes
        self.server_processes = []
        
        # Create service clients for both robots
        self.left_client = self.create_client(
            RemoteCmdInterface,
            '/fairino_robot1_command_service'
        )
        self.right_client = self.create_client(
            RemoteCmdInterface,
            '/fairino_robot2_command_service'
        )
        
        # Reuse services from demo.launch.py when they already exist.
        start_servers = self.get_parameter('start_hardware_servers').value
        if isinstance(start_servers, bool):
            start_servers = 'true' if start_servers else 'false'
        start_servers = str(start_servers).lower()

        services_available = self._command_services_available(timeout_sec=1.0)
        if start_servers in ('true', '1', 'yes'):
            self.start_hardware_servers()
        elif start_servers in ('false', '0', 'no'):
            self.get_logger().info("Using existing robot command services")
        elif services_available:
            self.get_logger().info("Found existing robot command services; not starting duplicate servers")
        else:
            self.start_hardware_servers()
        
        # Wait for services
        self.get_logger().info("Waiting for robot command services...")
        while not self.left_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Left robot service not available, waiting...')
        while not self.right_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Right robot service not available, waiting...')
        
        # Control parameters (matching joystick_controller.cpp)
        self.vel_percent = 30.0  # Velocity percentage (0-100)
        self.acc_percent = 30.0  # Acceleration percentage
        self.max_distance = 1000.0  # Maximum jog distance (mm) - allows longer continuous motion
        self.threshold = self.get_parameter('threshold').value
        self.use_keyboard = self.get_parameter('use_keyboard').value
        self.use_joy = self.get_parameter('use_joy').value
        self.joy_timeout_sec = self.get_parameter('joy_timeout_sec').value
        
        # Reference frame: 2 = Base/World coordinate system for Cartesian control
        self.ref_frame = 2
        
        # State tracking
        self.is_jogging = False
        self.current_key = None
        self.last_jog_time = time.time()
        self.last_jog_send_time = time.time()  # Track when we last sent StartJOG
        self.last_joy_msg = None
        self.last_joy_time = 0.0
        self.current_joy_motion = None
        self.joy_is_jogging = False
        self.last_joy_send_time = time.time()
        self.last_reset_button_state = False
        
        # Store current positions as home reference
        self.left_home_joint = None
        self.right_home_joint = None
        self.left_home_cart = None
        self.right_home_cart = None
        
        # Terminal settings
        self.is_tty = sys.stdin.isatty()
        if self.is_tty and self.use_keyboard:
            self.settings = termios.tcgetattr(sys.stdin)
        else:
            self.settings = None
            if self.use_keyboard:
                self.get_logger().warn("stdin is not a TTY - keyboard control disabled")

        if self.use_joy:
            joy_topic = self.get_parameter('joy_topic').value
            self.joy_sub = self.create_subscription(
                Joy,
                joy_topic,
                self.joy_callback,
                10,
            )
            self.joy_timer = self.create_timer(0.02, self.process_joy)
            self.get_logger().info(f"Listening for keyboard/gamepad input on {joy_topic}")
        
        # Initialize both robots
        self.initialize_robots()
        self.print_help()

    def _command_services_available(self, timeout_sec=0.0):
        """Return True when both robot command services are already present."""
        return (
            self.left_client.wait_for_service(timeout_sec=timeout_sec) and
            self.right_client.wait_for_service(timeout_sec=timeout_sec)
        )
    
    def start_hardware_servers(self):
        """Start the hardware server processes for both robots."""
        self.get_logger().info("🚀 Starting hardware servers...")
        
        # Get workspace root (3 levels up from build/fairino_hardware/...)
        workspace_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../..'))
        server_path = os.path.join(workspace_root, 'build/fairino_hardware/ros2_cmd_server')
        
        if not os.path.exists(server_path):
            self.get_logger().error(f"❌ Server executable not found: {server_path}")
            self.get_logger().error("   Build the project first: colcon build --packages-select fairino_hardware")
            sys.exit(1)
        
        robot1_ip = self.get_parameter('left_robot_ip').value
        robot2_ip = self.get_parameter('right_robot_ip').value
        
        # Start left robot server (Robot 1)
        self.get_logger().info(f"  Starting LEFT robot server ({robot1_ip})...")
        try:
            server1 = subprocess.Popen(
                [server_path, robot1_ip, '--ros-args',
                 '-r', 'fairino_remote_command_service:=fairino_robot1_command_service'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid  # Create new process group for clean shutdown
            )
            self.server_processes.append(server1)
            self.get_logger().info(f"  ✓ LEFT server started (PID: {server1.pid})")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to start LEFT server: {e}")
            sys.exit(1)
        
        # Start right robot server (Robot 2)
        self.get_logger().info(f"  Starting RIGHT robot server ({robot2_ip})...")
        try:
            server2 = subprocess.Popen(
                [server_path, robot2_ip, '--ros-args',
                 '-r', 'fairino_remote_command_service:=fairino_robot2_command_service'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid  # Create new process group for clean shutdown
            )
            self.server_processes.append(server2)
            self.get_logger().info(f"  ✓ RIGHT server started (PID: {server2.pid})")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to start RIGHT server: {e}")
            self.stop_hardware_servers()
            sys.exit(1)
        
        # Give servers time to initialize
        time.sleep(2.0)
        self.get_logger().info("✓ Hardware servers ready")
    
    def stop_hardware_servers(self):
        """Stop all hardware server processes."""
        if not self.server_processes:
            return
        
        self.get_logger().info("🛑 Stopping hardware servers...")
        for proc in self.server_processes:
            if proc.poll() is None:  # Process still running
                try:
                    # Send SIGTERM to process group
                    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                    proc.wait(timeout=2.0)
                    self.get_logger().info(f"  ✓ Server (PID: {proc.pid}) stopped")
                except subprocess.TimeoutExpired:
                    # Force kill if didn't stop gracefully
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                    proc.wait()
                    self.get_logger().warn(f"  ⚠ Server (PID: {proc.pid}) force killed")
                except Exception as e:
                    self.get_logger().error(f"  ❌ Error stopping server: {e}")
        
        self.server_processes.clear()
        self.get_logger().info("✓ All servers stopped")
    
    def initialize_robots(self):
        """Initialize both robots: reset errors, enable, and set manual mode."""
        self.get_logger().info("Initializing robots...")
        
        # Reset errors
        self.send_command(self.left_client, "ResetAllError()")
        self.send_command(self.right_client, "ResetAllError()")
        time.sleep(0.1)
        
        # Enable robots
        self.send_command(self.left_client, "RobotEnable(1)")
        self.send_command(self.right_client, "RobotEnable(1)")
        time.sleep(0.1)
        
        # Set manual mode (required for JOG)
        self.send_command(self.left_client, "Mode(0)")
        self.send_command(self.right_client, "Mode(0)")
        time.sleep(0.1)
        
        # Get and store current positions as home reference
        self.get_logger().info("Capturing current positions as home reference...")
        
        # Get left robot positions
        left_joint_str = self.send_command(self.left_client, "GetActualJointPosDegree(1)")
        left_cart_str = self.send_command(self.left_client, "GetActualTCPPose(0)")
        
        # Get right robot positions
        right_joint_str = self.send_command(self.right_client, "GetActualJointPosDegree(1)")
        right_cart_str = self.send_command(self.right_client, "GetActualTCPPose(0)")
        
        if left_joint_str and left_cart_str and right_joint_str and right_cart_str:
            self.left_home_joint = left_joint_str
            self.left_home_cart = left_cart_str
            self.right_home_joint = right_joint_str
            self.right_home_cart = right_cart_str
            
            self.get_logger().info(f"✓ Left robot home: {self.left_home_cart}")
            self.get_logger().info(f"✓ Right robot home: {self.right_home_cart}")
        else:
            self.get_logger().warn("⚠ Failed to get current positions")
        
        self.get_logger().info("✓ Both robots initialized and ready")
    
    def send_command(self, client, command):
        """Send a command to a robot and wait for response."""
        robot_name = "LEFT" if client == self.left_client else "RIGHT"
        self.get_logger().info(f"📤 [{robot_name}] Sending: {command}")
        
        request = RemoteCmdInterface.Request()
        request.cmd_str = command
        
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result() is not None:
            result = future.result().cmd_res
            self.get_logger().info(f"📥 [{robot_name}] Response: {result}")
            return result
        else:
            self.get_logger().warn(f"❌ [{robot_name}] Command failed: {command}")
            return None
    
    def _log_response(self, future, robot_name, command):
        """Callback to log async service responses."""
        try:
            result = future.result()
            if result is not None:
                self.get_logger().info(f"✅ [{robot_name}] Response: {result.cmd_res}")
            else:
                self.get_logger().warn(f"❌ [{robot_name}] No response for: {command}")
        except Exception as e:
            self.get_logger().error(f"❌ [{robot_name}] Exception: {str(e)}")
    
    def start_jog(self, client, axis, direction, vel_percent, log_command=True):
        """Send StartJOG command to a robot (async, non-blocking).
        
        Args:
            client: Service client for the robot
            axis: 1-6 for X,Y,Z,A,B,C
            direction: 0=negative, 1=positive
            vel_percent: Velocity percentage (0-100)
            log_command: Whether to log the command (False for repeated commands)
        """
        cmd = f"StartJOG(0,{axis},{direction},{vel_percent},{self.acc_percent},{self.max_distance})"
        robot_name = "LEFT" if client == self.left_client else "RIGHT"
        
        if log_command:
            self.get_logger().info(f"🔵 [{robot_name}] Sending: {cmd}")
        
        request = RemoteCmdInterface.Request()
        request.cmd_str = cmd
        
        # Send async with callback to log response
        future = client.call_async(request)
        if log_command:
            future.add_done_callback(lambda f: self._log_response(f, robot_name, cmd))
    
    def stop_jog(self):
        """Stop jogging on both robots (async, non-blocking)."""
        stop_cmd = "StopJOG(1)"  # Parameter 1 as per joystick_controller.cpp
        self.get_logger().info(f"⏹ Sending StopJOG to both robots")
        
        request_left = RemoteCmdInterface.Request()
        request_left.cmd_str = stop_cmd
        future_left = self.left_client.call_async(request_left)
        future_left.add_done_callback(lambda f: self._log_response(f, "LEFT", stop_cmd))
        
        request_right = RemoteCmdInterface.Request()
        request_right.cmd_str = stop_cmd
        future_right = self.right_client.call_async(request_right)
        future_right.add_done_callback(lambda f: self._log_response(f, "RIGHT", stop_cmd))

    def joy_callback(self, msg):
        """Store the latest Joy message from keyboard_joy_publisher or joy_node."""
        self.last_joy_msg = msg
        self.last_joy_time = time.time()

    def process_joy(self):
        """Convert /joy input to the same dual-arm StartJOG commands as keyboard input."""
        if not self.use_joy:
            return

        if not self.last_joy_msg or time.time() - self.last_joy_time > self.joy_timeout_sec:
            if self.joy_is_jogging:
                self.stop_jog()
                self.joy_is_jogging = False
                self.current_joy_motion = None
            return

        joy = self.last_joy_msg
        if len(joy.axes) < 5 or len(joy.buttons) < 8:
            return

        reset_pressed = joy.buttons[3] != 0
        if reset_pressed and not self.last_reset_button_state:
            self.get_logger().info("🔧 Resetting errors on both robots from /joy")
            request_left = RemoteCmdInterface.Request()
            request_left.cmd_str = "ResetAllError()"
            self.left_client.call_async(request_left)

            request_right = RemoteCmdInterface.Request()
            request_right.cmd_str = "ResetAllError()"
            self.right_client.call_async(request_right)
        self.last_reset_button_state = reset_pressed

        motion = self._motion_from_joy(joy)
        if not motion:
            if self.joy_is_jogging:
                self.stop_jog()
                self.joy_is_jogging = False
                self.current_joy_motion = None
            return

        now = time.time()
        should_send = (
            motion != self.current_joy_motion or
            now - self.last_joy_send_time > 0.1
        )
        if should_send:
            self._send_motion(motion, log_command=motion != self.current_joy_motion)
            self.current_joy_motion = motion
            self.last_joy_send_time = now

        self.joy_is_jogging = True

    def _motion_from_joy(self, joy):
        """Return (axis, left_dir, right_dir, velocity, label) from a Joy message."""
        if abs(joy.axes[0]) > self.threshold:
            direction = 1 if joy.axes[0] > 0.0 else 0
            label = "X+" if direction == 1 else "X-"
            return (1, direction, direction, abs(joy.axes[0]) * self.vel_percent, label)

        if abs(joy.axes[1]) > self.threshold:
            left_dir = 1 if joy.axes[1] > 0.0 else 0
            right_dir = 0 if joy.axes[1] > 0.0 else 1
            label = "Y+" if joy.axes[1] > 0.0 else "Y-"
            return (2, left_dir, right_dir, abs(joy.axes[1]) * self.vel_percent, label)

        if abs(joy.axes[4]) > self.threshold:
            left_dir = 0 if joy.axes[4] > 0.0 else 1
            right_dir = 1 if joy.axes[4] > 0.0 else 0
            label = "Z-" if joy.axes[4] > 0.0 else "Z+"
            return (3, left_dir, right_dir, abs(joy.axes[4]) * self.vel_percent, label)

        if joy.buttons[5]:
            return (4, 1, 0, self.vel_percent, "A+")
        if joy.buttons[7]:
            return (4, 0, 1, self.vel_percent, "A-")
        if joy.buttons[4]:
            return (5, 1, 0, self.vel_percent, "B+")
        if joy.buttons[6]:
            return (5, 0, 1, self.vel_percent, "B-")
        if joy.buttons[2]:
            return (6, 1, 1, self.vel_percent, "C+")
        if joy.buttons[0]:
            return (6, 0, 0, self.vel_percent, "C-")

        return None

    def _send_motion(self, motion, log_command=True):
        axis, left_dir, right_dir, velocity, label = motion
        if log_command:
            self.get_logger().info(f"🎮 /joy motion: {label}")
        self.start_jog(self.left_client, axis, left_dir, velocity, log_command=log_command)
        self.start_jog(self.right_client, axis, right_dir, velocity, log_command=log_command)
    
    def print_help(self):
        """Print control instructions."""
        self.get_logger().info("=" * 70)
        self.get_logger().info("  DUAL ARM DIRECT JOG CONTROL")
        self.get_logger().info("  Using StartJOG/StopJOG commands (no MoveIt)")
        self.get_logger().info("  Inputs: terminal keyboard and/or /joy")
        self.get_logger().info("=" * 70)
        self.get_logger().info("")
        self.get_logger().info("📍 TRANSLATION (Cartesian):")
        self.get_logger().info("  a/d     - X-axis: Left↔Right (MIRRORED)")
        self.get_logger().info("  w/s     - Y-axis: Forward↔Backward (SAME)")
        self.get_logger().info("  q/e     - Z-axis: Down↔Up (SAME)")
        self.get_logger().info("")
        self.get_logger().info("🔄 ROTATION (Cartesian):")
        self.get_logger().info("  j/l     - Roll/A (SAME)")
        self.get_logger().info("  i/k     - Pitch/B (SAME)")
        self.get_logger().info("  u/o     - Yaw/C (MIRRORED)")
        self.get_logger().info("")
        self.get_logger().info("⚙️  CONTROL:")
        self.get_logger().info("  r       - Reset errors on both robots")
        self.get_logger().info("  h       - Home both robots")
        self.get_logger().info(f"  +/-     - Adjust speed (current: {self.vel_percent}%)")
        self.get_logger().info("  SPACE   - Stop current motion")
        self.get_logger().info("  ESC     - Quit")
        self.get_logger().info("=" * 70)
        self.get_logger().info("✅ Ready! Press keys to start motion, SPACE to stop.")
        self.get_logger().info("=" * 70)
    
    def get_key(self, timeout=0.02):  # Faster polling for better responsiveness
        """Get single keypress without blocking."""
        if not self.is_tty:
            return ''
        
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def process_key(self, key):
        """Process keyboard input and send JOG commands."""
        # If no new key, check if we should continue or timeout
        if not key:
            if self.is_jogging and self.current_key:
                # Check timeout - if key not pressed for 50ms, assume released and stop immediately
                if time.time() - self.last_jog_time > 0.05:
                    self.get_logger().info("⏹ Key released - stopping motion immediately")
                    self.stop_jog()
                    self.is_jogging = False
                    self.current_key = None
                else:
                    # Keep sending StartJOG commands to maintain continuous motion
                    # Send new command every 100ms
                    if time.time() - self.last_jog_send_time > 0.1:
                        self._send_current_jog_command()
                        self.last_jog_send_time = time.time()
            return True
        
        # Key was pressed - update timing
        self.last_jog_time = time.time()
        
        # Handle ESC
        if key == '\x1b':
            self.get_logger().info("ESC pressed - shutting down...")
            if self.is_jogging:
                self.stop_jog()
            return False
        
        # Handle SPACE to stop current motion
        if key == ' ':
            if self.is_jogging:
                self.get_logger().info("⏹ SPACE pressed - stopping motion")
                self.stop_jog()
                self.is_jogging = False
                self.current_key = None
            return True
        
        # Handle special commands (one-shot)
        if key == 'r':
            self.get_logger().info("🔧 Resetting errors on both robots...")
            self.send_command(self.left_client, "ResetAllError()")
            self.send_command(self.right_client, "ResetAllError()")
            return True
        
        if key == 'h':
            self.get_logger().info("🏠 Homing both robots...")
            self.stop_jog()
            # Send home command (adjust if your robots have different home command)
            self.send_command(self.left_client, "MoveJ(jointPos,descPos,0,0,100.0,100.0,100.0,-1.0,exAxisPos)")
            self.send_command(self.right_client, "MoveJ(jointPos,descPos,0,0,100.0,100.0,100.0,-1.0,exAxisPos)")
            return True
        
        if key == '+' or key == '=':
            self.vel_percent = min(100.0, self.vel_percent + 5.0)
            self.get_logger().info(f"⚡ Speed: {self.vel_percent}%")
            return True
        
        if key == '-' or key == '_':
            self.vel_percent = max(5.0, self.vel_percent - 5.0)
            self.get_logger().info(f"⚡ Speed: {self.vel_percent}%")
            return True
        
        # If same key as before, update timing and continue (already jogging)
        if key == self.current_key and self.is_jogging:
            # Just update timing to reset timeout
            return True
        
        # Different key - stop previous motion first
        if self.is_jogging and key != self.current_key:
            self.get_logger().info("⏹ Stopping previous motion")
            self.stop_jog()
            time.sleep(0.1)  # Brief pause between direction changes
        
        # Store current key and update timing
        self.current_key = key
        self.last_jog_send_time = time.time()
        
        # Process motion keys
        jogging = False
        
        # X-axis (SAME)
        if key == 'a':
            self.get_logger().info("\n◀◀◀ Moving X- (both -X) ◀◀◀")
            self.start_jog(self.left_client, 1, 0, self.vel_percent)   # Left: -X
            self.start_jog(self.right_client, 1, 0, self.vel_percent)  # Right: -X (same)
            jogging = True
        elif key == 'd':
            self.get_logger().info("\n▶▶▶ Moving X+ (both +X) ▶▶▶")
            self.start_jog(self.left_client, 1, 1, self.vel_percent)   # Left: +X
            self.start_jog(self.right_client, 1, 1, self.vel_percent)  # Right: +X (same)
            jogging = True
        
        # Y-axis (MIRRORED)
        elif key == 'w':
            self.get_logger().info("\n⬆⬆⬆ Moving Y+ SYMMETRIC (toward/away center) ⬆⬆⬆")
            self.start_jog(self.left_client, 2, 1, self.vel_percent)   # Left: +Y
            self.start_jog(self.right_client, 2, 0, self.vel_percent)  # Right: -Y (mirrored)
            jogging = True
        elif key == 's':
            self.get_logger().info("\n⬇⬇⬇ Moving Y- SYMMETRIC (toward/away center) ⬇⬇⬇")
            self.start_jog(self.left_client, 2, 0, self.vel_percent)   # Left: -Y
            self.start_jog(self.right_client, 2, 1, self.vel_percent)  # Right: +Y (mirrored)
            jogging = True
        
        # Z-axis (MIRRORED)
        elif key == 'q':
            self.get_logger().info("\n⬆⬆⬆ Moving Z+ SYMMETRIC (toward/away center) ⬆⬆⬆")
            self.start_jog(self.left_client, 3, 1, self.vel_percent)   # Left: +Z
            self.start_jog(self.right_client, 3, 0, self.vel_percent)  # Right: -Z (mirrored)
            jogging = True
        elif key == 'e':
            self.get_logger().info("\n⬇⬇⬇ Moving Z- SYMMETRIC (toward/away center) ⬇⬇⬇")
            self.start_jog(self.left_client, 3, 0, self.vel_percent)   # Left: -Z
            self.start_jog(self.right_client, 3, 1, self.vel_percent)  # Right: +Z (mirrored)
            jogging = True
        
        # Roll/A (MIRRORED)
        elif key == 'l':
            self.start_jog(self.left_client, 4, 1, self.vel_percent)
            self.start_jog(self.right_client, 4, 0, self.vel_percent)  # Mirrored
            jogging = True
        elif key == 'j':
            self.start_jog(self.left_client, 4, 0, self.vel_percent)
            self.start_jog(self.right_client, 4, 1, self.vel_percent)  # Mirrored
            jogging = True
        
        # Pitch/B (MIRRORED)
        elif key == 'i':
            self.start_jog(self.left_client, 5, 1, self.vel_percent)
            self.start_jog(self.right_client, 5, 0, self.vel_percent)  # Mirrored
            jogging = True
        elif key == 'k':
            self.start_jog(self.left_client, 5, 0, self.vel_percent)
            self.start_jog(self.right_client, 5, 1, self.vel_percent)  # Mirrored
            jogging = True
        
        # Yaw/C (SAME)
        elif key == 'o':
            self.start_jog(self.left_client, 6, 1, self.vel_percent)   # Left: +C
            self.start_jog(self.right_client, 6, 1, self.vel_percent)  # Right: +C (same)
            jogging = True
        elif key == 'u':
            self.start_jog(self.left_client, 6, 0, self.vel_percent)   # Left: -C
            self.start_jog(self.right_client, 6, 0, self.vel_percent)  # Right: -C (same)
            jogging = True
        
        self.is_jogging = jogging
        
        # If we didn't recognize the key as a motion key, clear current_key
        if not jogging:
            self.current_key = None
        
        return True
    
    def _send_current_jog_command(self):
        """Resend the current JOG command to maintain continuous motion."""
        if not self.current_key:
            return
        
        # Resend the StartJOG command for current key (without logging)
        key = self.current_key
        
        # X-axis (SAME)
        if key == 'a':
            self.start_jog(self.left_client, 1, 0, self.vel_percent, log_command=False)
            self.start_jog(self.right_client, 1, 0, self.vel_percent, log_command=False)
        elif key == 'd':
            self.start_jog(self.left_client, 1, 1, self.vel_percent, log_command=False)
            self.start_jog(self.right_client, 1, 1, self.vel_percent, log_command=False)
        # Y-axis (MIRRORED)
        elif key == 'w':
            self.start_jog(self.left_client, 2, 1, self.vel_percent, log_command=False)
            self.start_jog(self.right_client, 2, 0, self.vel_percent, log_command=False)
        elif key == 's':
            self.start_jog(self.left_client, 2, 0, self.vel_percent, log_command=False)
            self.start_jog(self.right_client, 2, 1, self.vel_percent, log_command=False)
        # Z-axis (MIRRORED)
        elif key == 'q':
            self.start_jog(self.left_client, 3, 1, self.vel_percent, log_command=False)
            self.start_jog(self.right_client, 3, 0, self.vel_percent, log_command=False)
        elif key == 'e':
            self.start_jog(self.left_client, 3, 0, self.vel_percent, log_command=False)
            self.start_jog(self.right_client, 3, 1, self.vel_percent, log_command=False)
        # Roll/A (MIRRORED)
        elif key == 'l':
            self.start_jog(self.left_client, 4, 1, self.vel_percent, log_command=False)
            self.start_jog(self.right_client, 4, 0, self.vel_percent, log_command=False)
        elif key == 'j':
            self.start_jog(self.left_client, 4, 0, self.vel_percent, log_command=False)
            self.start_jog(self.right_client, 4, 1, self.vel_percent, log_command=False)
        # Pitch/B (MIRRORED)
        elif key == 'i':
            self.start_jog(self.left_client, 5, 1, self.vel_percent, log_command=False)
            self.start_jog(self.right_client, 5, 0, self.vel_percent, log_command=False)
        elif key == 'k':
            self.start_jog(self.left_client, 5, 0, self.vel_percent, log_command=False)
            self.start_jog(self.right_client, 5, 1, self.vel_percent, log_command=False)
        # Yaw/C (SAME)
        elif key == 'o':
            self.start_jog(self.left_client, 6, 1, self.vel_percent, log_command=False)
            self.start_jog(self.right_client, 6, 1, self.vel_percent, log_command=False)
        elif key == 'u':
            self.start_jog(self.left_client, 6, 0, self.vel_percent, log_command=False)
            self.start_jog(self.right_client, 6, 0, self.vel_percent, log_command=False)
    
    def run(self):
        """Main control loop."""
        try:
            while rclpy.ok():
                if self.use_keyboard and self.is_tty:
                    key = self.get_key()
                    if not self.process_key(key):
                        break
                rclpy.spin_once(self, timeout_sec=0.01)
        finally:
            # Cleanup
            self.stop_jog()
            if self.is_tty and self.settings:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            self.stop_hardware_servers()
    
    def __del__(self):
        """Cleanup on destruction."""
        if hasattr(self, 'is_tty') and self.is_tty and hasattr(self, 'settings') and self.settings:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            except:
                pass
        
        # Stop hardware servers
        if hasattr(self, 'server_processes'):
            self.stop_hardware_servers()


def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = DualArmsJogTeleop()
        if node.is_tty or node.use_joy:
            node.run()
        else:
            node.get_logger().error("Cannot run without TTY unless use_joy is enabled")
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
