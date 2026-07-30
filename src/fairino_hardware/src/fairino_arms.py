#!/usr/bin/env python3
"""
Fairino Dual/Single Arm Controller
Reuses exact command logic from keyboard_joy_publisher.py
Supports switching between dual-arm and single-arm control modes

Press 'c' to toggle dual/single mode
Press 'x' to switch between left/right arms in single mode

Key Mapping (same as keyboard_joy_publisher.py):
  Translation:
    a/d     → X-axis (left/right)
    w/s     → Y-axis (forward/back)
    q/e     → Z-axis (down/up)
  
  Rotation:
    j/l     → A rotation (roll)
    i/k     → B rotation (pitch)
    u/o     → C rotation (yaw)

Author: Based on keyboard_joy_publisher.py
"""

import rclpy
from rclpy.node import Node
from fairino_msgs.srv import RemoteCmdInterface
import sys
import termios
import tty
import select
import subprocess
import signal
import os
import time


class FairinoArmsController(Node):
    """Multi-mode controller for Fairino dual-arm robot."""
    
    # Control modes
    MODE_DUAL = "DUAL"
    MODE_SINGLE = "SINGLE"
    
    # Active arms in single mode
    ARM_LEFT = "LEFT"
    ARM_RIGHT = "RIGHT"
    
    def __init__(self):
        super().__init__('fairino_arms_controller')
        
        # Control parameters
        self.vel_percent = 30.0
        self.acc_percent = 30.0
        self.max_distance = 1000.0
        
        # State management
        self.mode = self.MODE_DUAL
        self.active_arm = self.ARM_LEFT
        self.current_key = None
        self.is_jogging = False
        
        # Key hold logic (same as keyboard_joy_publisher.py)
        self.key_hold_cycles = 0
        self.KEY_HOLD_DURATION = 10  # Hold key for 10 cycles after detection
        
        # Timing parameters
        self.jog_resend_interval = 0.1   # 100ms (10Hz publish rate like keyboard_joy_publisher)
        
        # Terminal settings
        self.is_tty = sys.stdin.isatty()
        self.settings = None
        if self.is_tty:
            self.settings = termios.tcgetattr(sys.stdin)
        
        # Start hardware servers
        self.server_processes = []
        self.start_hardware_servers()
        
        # Wait for servers to be ready
        time.sleep(2)
        
        # Create service clients
        self.left_client = self.create_client(
            RemoteCmdInterface, 
            '/fairino_robot1_command_service'
        )
        self.right_client = self.create_client(
            RemoteCmdInterface, 
            '/fairino_robot2_command_service'
        )
        
        # Wait for services
        self.get_logger().info("Waiting for robot services...")
        while not self.left_client.wait_for_service(timeout_sec=1.0) or \
              not self.right_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Services not available, waiting...")
        
        self.get_logger().info("✓ Services connected")
        
        # Initialize robots
        self.initialize_robots()
        
        # Print help
        self.print_help()
    
    def start_hardware_servers(self):
        """Start hardware command servers for both robots."""
        robot_ips = ['192.168.58.2', '192.168.58.3']
        service_names = ['fairino_robot1_command_service', 'fairino_robot2_command_service']
        
        server_path = os.path.join(
            os.getcwd(), 
            'build/fairino_hardware/ros2_cmd_server'
        )
        
        if not os.path.exists(server_path):
            self.get_logger().error(f"Server not found at {server_path}")
            return
        
        for ip, service in zip(robot_ips, service_names):
            cmd = [
                server_path, ip,
                '--ros-args',
                '-r', f'fairino_remote_command_service:={service}'
            ]
            
            try:
                process = subprocess.Popen(
                    cmd,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL
                )
                self.server_processes.append(process)
                self.get_logger().info(f"Started server for {ip} -> {service}")
            except Exception as e:
                self.get_logger().error(f"Failed to start server for {ip}: {e}")
    
    def stop_hardware_servers(self):
        """Stop all hardware servers."""
        for process in self.server_processes:
            try:
                process.send_signal(signal.SIGTERM)
                process.wait(timeout=2)
            except:
                process.kill()
        self.server_processes.clear()
    
    def initialize_robots(self):
        """Initialize both robots."""
        # Reset errors
        self.send_command(self.left_client, "ResetAllError()")
        self.send_command(self.right_client, "ResetAllError()")
        time.sleep(0.1)
        
        # Enable robots
        self.send_command(self.left_client, "RobotEnable(1)")
        self.send_command(self.right_client, "RobotEnable(1)")
        time.sleep(0.1)
        
        # Set manual mode
        self.send_command(self.left_client, "Mode(0)")
        self.send_command(self.right_client, "Mode(0)")
        
        self.get_logger().info("✓ Robots initialized")
    
    def send_command(self, client, cmd_str):
        """Send command to robot."""
        request = RemoteCmdInterface.Request()
        request.cmd_str = cmd_str
        future = client.call_async(request)
        return future
    
    def start_jog(self, client, axis, direction, vel_percent, log_command=True):
        """Start JOG motion on specified axis."""
        cmd = f"StartJOG(0,{axis},{direction},{vel_percent},{self.acc_percent},{self.max_distance})"
        if log_command:
            robot = "LEFT" if client == self.left_client else "RIGHT"
            self.get_logger().info(f"[{robot}] {cmd}")
        self.send_command(client, cmd)
    
    def stop_jog(self):
        """Stop JOG motion on all robots."""
        if self.mode == self.MODE_DUAL or self.active_arm == self.ARM_LEFT:
            self.send_command(self.left_client, "StopJOG(1)")
        if self.mode == self.MODE_DUAL or self.active_arm == self.ARM_RIGHT:
            self.send_command(self.right_client, "StopJOG(1)")
        
        self.is_jogging = False
        self.current_key = None
    
    def toggle_mode(self):
        """Toggle between dual and single arm modes."""
        self.stop_jog()
        
        if self.mode == self.MODE_DUAL:
            self.mode = self.MODE_SINGLE
            self.get_logger().info(f"\n{'='*50}")
            self.get_logger().info(f"   MODE: SINGLE ARM ({self.active_arm})")
            self.get_logger().info(f"{'='*50}")
        else:
            self.mode = self.MODE_DUAL
            self.get_logger().info(f"\n{'='*50}")
            self.get_logger().info(f"   MODE: DUAL ARM")
            self.get_logger().info(f"{'='*50}")
    
    def switch_arm(self):
        """Switch active arm in single mode."""
        if self.mode == self.MODE_SINGLE:
            self.stop_jog()
            
            if self.active_arm == self.ARM_LEFT:
                self.active_arm = self.ARM_RIGHT
            else:
                self.active_arm = self.ARM_LEFT
            
            self.get_logger().info(f"\n{'='*50}")
            self.get_logger().info(f"   ACTIVE ARM: {self.active_arm}")
            self.get_logger().info(f"{'='*50}")
    
    def print_help(self):
        """Print control help."""
        self.get_logger().info("\n" + "="*70)
        self.get_logger().info("  FAIRINO DUAL/SINGLE ARM CONTROLLER")
        self.get_logger().info("  (Using keyboard_joy_publisher.py key mappings)")
        self.get_logger().info("="*70)
        self.get_logger().info(f"  Current Mode: {self.mode}")
        if self.mode == self.MODE_SINGLE:
            self.get_logger().info(f"  Active Arm: {self.active_arm}")
        self.get_logger().info("="*70)
        self.get_logger().info("  MODE CONTROL:")
        self.get_logger().info("    c - Toggle DUAL/SINGLE mode")
        self.get_logger().info("    x - Switch LEFT/RIGHT arm (single mode only)")
        self.get_logger().info("    r - Reset all errors on both robots")
        self.get_logger().info("")
        self.get_logger().info("  📍 TRANSLATION:")
        self.get_logger().info("    a/d - X-axis (left/right)")
        self.get_logger().info("    w/s - Y-axis (forward/backward)")
        self.get_logger().info("    q/e - Z-axis (down/up)")
        self.get_logger().info("")
        self.get_logger().info("  🔄 ROTATION:")
        self.get_logger().info("    j/l - A rotation (roll)")
        self.get_logger().info("    i/k - B rotation (pitch)")
        self.get_logger().info("    u/o - C rotation (yaw)")
        self.get_logger().info("")
        self.get_logger().info("  ⚙️  CONTROL:")
        self.get_logger().info("    ESC - Quit")
        self.get_logger().info("="*70)
        if self.mode == self.MODE_DUAL:
            self.get_logger().info("  DUAL MODE: All axes mirrored for synchronized motion")
            self.get_logger().info("  (Both arms move in same physical direction)")
        else:
            self.get_logger().info(f"  SINGLE MODE: Only {self.active_arm} arm moves")
        self.get_logger().info("="*70 + "\n")
    
    def get_key(self, timeout=0.1):
        """Get single keypress without blocking (same as keyboard_joy_publisher.py)."""
        if not self.is_tty:
            return ''
        
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def publish_jog(self):
        """Read keyboard and send JOG commands (like publish_joy in keyboard_joy_publisher.py)."""
        # Poll for key
        key = self.get_key(timeout=0.1)
        
        # Update key state with hold logic (same as keyboard_joy_publisher.py)
        if key:
            # New key detected - update and reset hold counter
            if key != '\x1b':  # Don't hold ESC key
                self.current_key = key
                self.key_hold_cycles = self.KEY_HOLD_DURATION
        else:
            # No key detected - decrement hold counter
            if self.key_hold_cycles > 0:
                self.key_hold_cycles -= 1
            else:
                self.current_key = None
        
        # Use current_key (held key) for JOG commands
        active_key = self.current_key
        
        # Determine which arms to control
        if self.mode == self.MODE_DUAL:
            left_active = True
            right_active = True
        else:
            left_active = (self.active_arm == self.ARM_LEFT)
            right_active = (self.active_arm == self.ARM_RIGHT)
        
        # Process active key and send JOG commands
        # Using EXACT same mappings as keyboard_joy_publisher.py
        jogging = False
        
        # X-axis translation: a/d → axis 0 (left stick X)
        # a = -1.0 → direction 0, d = 1.0 → direction 1
        if active_key == 'a':
            # In dual mode: MIRRORED (left dir 0, right dir 1)
            # In single mode: SAME (selected arm dir 0)
            if left_active:
                self.start_jog(self.left_client, 1, 0, self.vel_percent, log_command=True)
            if right_active:
                dir_right = 1 if self.mode == self.MODE_DUAL else 0
                self.start_jog(self.right_client, 1, dir_right, self.vel_percent, log_command=True)
            jogging = True
            
        elif active_key == 'd':
            if left_active:
                self.start_jog(self.left_client, 1, 1, self.vel_percent, log_command=True)
            if right_active:
                dir_right = 0 if self.mode == self.MODE_DUAL else 1
                self.start_jog(self.right_client, 1, dir_right, self.vel_percent, log_command=True)
            jogging = True
        
        # Y-axis translation: w/s → axis 1 (left stick Y)
        # w = 1.0 → direction 1, s = -1.0 → direction 0
        elif active_key == 'w':
            # Dual mode: MIRRORED (left dir 1, right dir 0) for same physical direction
            if left_active:
                self.start_jog(self.left_client, 2, 1, self.vel_percent, log_command=True)
            if right_active:
                dir_right = 0 if self.mode == self.MODE_DUAL else 1
                self.start_jog(self.right_client, 2, dir_right, self.vel_percent, log_command=True)
            jogging = True
            
        elif active_key == 's':
            # Dual mode: MIRRORED (left dir 0, right dir 1) for same physical direction
            if left_active:
                self.start_jog(self.left_client, 2, 0, self.vel_percent, log_command=True)
            if right_active:
                dir_right = 1 if self.mode == self.MODE_DUAL else 0
                self.start_jog(self.right_client, 2, dir_right, self.vel_percent, log_command=True)
            jogging = True
        
        # Z-axis translation: q/e → axis 4 (right stick Y)
        # q = -1.0 → direction 0, e = 1.0 → direction 1
        elif active_key == 'q':
            # Dual mode: MIRRORED (left dir 0, right dir 1) for same physical direction
            if left_active:
                self.start_jog(self.left_client, 3, 0, self.vel_percent, log_command=True)
            if right_active:
                dir_right = 1 if self.mode == self.MODE_DUAL else 0
                self.start_jog(self.right_client, 3, dir_right, self.vel_percent, log_command=True)
            jogging = True
            
        elif active_key == 'e':
            # Dual mode: MIRRORED (left dir 1, right dir 0) for same physical direction
            if left_active:
                self.start_jog(self.left_client, 3, 1, self.vel_percent, log_command=True)
            if right_active:
                dir_right = 0 if self.mode == self.MODE_DUAL else 1
                self.start_jog(self.right_client, 3, dir_right, self.vel_percent, log_command=True)
            jogging = True
        
        # A rotation (Roll): j/l → R1/R2 buttons
        # l = R1 → direction 1, j = R2 → direction 0
        elif active_key == 'l':
            # Dual mode: MIRRORED (left dir 1, right dir 0) for same physical direction
            if left_active:
                self.start_jog(self.left_client, 4, 1, self.vel_percent, log_command=True)
            if right_active:
                dir_right = 0 if self.mode == self.MODE_DUAL else 1
                self.start_jog(self.right_client, 4, dir_right, self.vel_percent, log_command=True)
            jogging = True
            
        elif active_key == 'j':
            # Dual mode: MIRRORED (left dir 0, right dir 1) for same physical direction
            if left_active:
                self.start_jog(self.left_client, 4, 0, self.vel_percent, log_command=True)
            if right_active:
                dir_right = 1 if self.mode == self.MODE_DUAL else 0
                self.start_jog(self.right_client, 4, dir_right, self.vel_percent, log_command=True)
            jogging = True
        
        # B rotation (Pitch): i/k → L1/L2 buttons
        # i = L1 → direction 1, k = L2 → direction 0
        elif active_key == 'i':
            # Dual mode: MIRRORED (left dir 1, right dir 0) for same physical direction
            if left_active:
                self.start_jog(self.left_client, 5, 1, self.vel_percent, log_command=True)
            if right_active:
                dir_right = 0 if self.mode == self.MODE_DUAL else 1
                self.start_jog(self.right_client, 5, dir_right, self.vel_percent, log_command=True)
            jogging = True
            
        elif active_key == 'k':
            # Dual mode: MIRRORED (left dir 0, right dir 1) for same physical direction
            if left_active:
                self.start_jog(self.left_client, 5, 0, self.vel_percent, log_command=True)
            if right_active:
                dir_right = 1 if self.mode == self.MODE_DUAL else 0
                self.start_jog(self.right_client, 5, dir_right, self.vel_percent, log_command=True)
            jogging = True
        
        # C rotation (Yaw): u/o → Circle/Square buttons
        # o = Circle → direction 1, u = Square → direction 0
        elif active_key == 'o':
            # Dual mode: SAME direction (both dir 1)
            if left_active:
                self.start_jog(self.left_client, 6, 1, self.vel_percent, log_command=True)
            if right_active:
                self.start_jog(self.right_client, 6, 1, self.vel_percent, log_command=True)
            jogging = True
            
        elif active_key == 'u':
            # Dual mode: SAME direction (both dir 0)
            if left_active:
                self.start_jog(self.left_client, 6, 0, self.vel_percent, log_command=True)
            if right_active:
                self.start_jog(self.right_client, 6, 0, self.vel_percent, log_command=True)
            jogging = True
        
        # Mode control keys (one-shot)
        elif active_key == 'c':
            self.toggle_mode()
            self.current_key = None  # Clear immediately
            jogging = False
            
        elif active_key == 'x':
            self.switch_arm()
            self.current_key = None  # Clear immediately
            jogging = False
        
        # Error reset key (one-shot)
        elif active_key == 'r':
            self.get_logger().info("🔧 [RESET] Resetting errors on both robots")
            self.send_command(self.left_client, "ResetAllError()")
            self.send_command(self.right_client, "ResetAllError()")
            self.current_key = None  # Clear immediately
            jogging = False
        
        # Stop motion if not jogging
        if not jogging and self.is_jogging:
            self.stop_jog()
        
        self.is_jogging = jogging
        
        # Handle ESC separately (immediate action)
        if key == '\x1b':
            self.get_logger().info("\nESC pressed - shutting down...")
            if self.is_tty and self.settings:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            rclpy.shutdown()
    
    def run(self):
        """Main control loop using timer (like keyboard_joy_publisher.py)."""
        # Create timer to publish at 10Hz (same as keyboard_joy_publisher.py)
        self.timer = self.create_timer(0.1, self.publish_jog)
        
        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            pass
        finally:
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
        
        if hasattr(self, 'server_processes'):
            self.stop_hardware_servers()


def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = FairinoArmsController()
        if node.is_tty:
            node.run()
        else:
            node.get_logger().error("Cannot run without TTY")
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
