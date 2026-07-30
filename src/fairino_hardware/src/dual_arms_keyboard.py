#!/usr/bin/env python3
"""
Dual-Arm Keyboard to Joy Publisher
Converts keyboard input to Joy messages for dual-arm mirrored Cartesian control.

Keyboard Mapping (WASD Gaming Style):
  Cartesian Translation:
    a/d     → X-axis (left/right) - MIRRORED
    w/s     → Y-axis (forward/back) - SAME
    q/e     → Z-axis (up/down) - SAME
  
  Cartesian Rotation (mirrored between arms):
    j/l     → A rotation (roll) - SAME
    i/k     → B rotation (pitch) - MIRRORED
    u/o     → C rotation (yaw) - MIRRORED
  
  Control:
    r       → Reset errors on both robots
    c       → Cycle control mode: both → left → right → both
    ESC     → Quit

Author: GitHub Copilot
Date: March 27, 2026
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import sys
import select
import termios
import tty


class DualArmKeyboardJoyPublisher(Node):
    def __init__(self):
        super().__init__('dual_arms_keyboard_joy_publisher')

        self.declare_parameter('joy_topic', '/joy_keyboard')
        joy_topic = self.get_parameter('joy_topic').value
        
        # Publisher to a keyboard-specific Joy topic. A mux can forward it to /joy.
        self.joy_pub = self.create_publisher(Joy, joy_topic, 10)
        
        # Publish rate: 10Hz to match joystick controller's control rate
        self.timer = self.create_timer(0.1, self.publish_joy)
        
        # Current key pressed and hold counter
        self.current_key = None
        self.key_hold_cycles = 0
        self.KEY_HOLD_DURATION = 5  # Hold key for 5 cycles (100ms) after detection
        
        # Check if stdin is a terminal (TTY)
        self.is_tty = sys.stdin.isatty()
        
        # Terminal settings for raw keyboard input (only if TTY)
        if self.is_tty:
            self.settings = termios.tcgetattr(sys.stdin)
            self.print_help()
        else:
            self.settings = None
            self.get_logger().warn("Keyboard input disabled - stdin is not a TTY")
    
    def print_help(self):
        """Print control instructions."""
        self.get_logger().info("=" * 70)
        self.get_logger().info("  DUAL ARM MIRRORED KEYBOARD CONTROL")
        self.get_logger().info("  CARTESIAN COORDINATE CONTROL MODE")
        self.get_logger().info("=" * 70)
        self.get_logger().info("")
        self.get_logger().info("📍 TRANSLATION (Cartesian Coordinates):")
        self.get_logger().info("  a/d     - X-axis: Left↔Right (MIRRORED)")
        self.get_logger().info("  w/s     - Y-axis: Forward↔Backward (SAME)")
        self.get_logger().info("  q/e     - Z-axis: Down↔Up (SAME)")
        self.get_logger().info("")
        self.get_logger().info("🔄 ROTATION (Cartesian Coordinates):")
        self.get_logger().info("  j/l     - A rotation: Roll (SAME)")
        self.get_logger().info("  i/k     - B rotation: Pitch (MIRRORED)")
        self.get_logger().info("  u/o     - C rotation: Yaw (MIRRORED)")
        self.get_logger().info("")
        self.get_logger().info("⚙️  CONTROL:")
        self.get_logger().info("  r       - Reset errors on both robots")
        self.get_logger().info("  c       - Cycle mode: both → left → right → both")
        self.get_logger().info("  ESC     - Quit")
        self.get_logger().info("=" * 70)
        self.get_logger().info("✓ Ready! Hold keys to move, release to stop.")
        self.get_logger().info("  X-axis mirrored, Y/Z synchronous, pitch/yaw mirrored, roll same")
        self.get_logger().info("=" * 70)
    
    def get_key(self, timeout=0.1):
        """Get single keypress without blocking."""
        if not self.is_tty:
            return ''
        
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def publish_joy(self):
        """Read keyboard and publish Joy message with key hold logic."""
        # Poll for key
        key = self.get_key(timeout=0.1)
        
        # Update key state with hold logic
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
        
        # Use current_key (held key) for Joy message
        active_key = self.current_key
        
        # Create Joy message with same structure as joystick
        # axes: [left_x, left_y, left_trigger, right_x, right_y, right_trigger, dpad_x, dpad_y]
        # buttons: [square, X, circle, triangle, L1, R1, L2, R2, share, options, L3, R3, PS]
        joy_msg = Joy()
        joy_msg.header.stamp = self.get_clock().now().to_msg()
        joy_msg.axes = [0.0] * 8      # 8 axes, all zero by default
        joy_msg.buttons = [0] * 13    # 13 buttons, all zero by default
        
        # Map keyboard to joystick axes/buttons
        # Translation keys map to axes with full deflection (±1.0)
        if active_key == 'a':
            joy_msg.axes[0] = -1.0  # Left stick X negative (left)
        elif active_key == 'd':
            joy_msg.axes[0] = 1.0   # Left stick X positive (right)
        elif active_key == 'w':
            joy_msg.axes[1] = 1.0   # Left stick Y positive (forward)
        elif active_key == 's':
            joy_msg.axes[1] = -1.0  # Left stick Y negative (backward)
        elif active_key == 'q':
            joy_msg.axes[4] = -1.0  # Right stick Y negative (down)
        elif active_key == 'e':
            joy_msg.axes[4] = 1.0   # Right stick Y positive (up)
        
        # Rotation keys map to buttons (R1/R2, L1/L2, Circle/Square)
        elif active_key == 'l':
            joy_msg.buttons[5] = 1  # R1 button (+A rotation)
        elif active_key == 'j':
            joy_msg.buttons[7] = 1  # R2 button (-A rotation)
        elif active_key == 'i':
            joy_msg.buttons[4] = 1  # L1 button (+B rotation)
        elif active_key == 'k':
            joy_msg.buttons[6] = 1  # L2 button (-B rotation)
        elif active_key == 'o':
            joy_msg.buttons[2] = 1  # Circle button (+C rotation)
        elif active_key == 'u':
            joy_msg.buttons[0] = 1  # Square button (-C rotation)
        
        # Control keys (one-shot, don't hold)
        elif active_key == 'r':
            joy_msg.buttons[3] = 1  # Triangle button (reset errors)
            self.get_logger().info("🔧 [RESET] Resetting errors on both robots")
            self.current_key = None  # Clear immediately for one-shot action
        elif active_key == 'c':
            joy_msg.buttons[1] = 1  # X button: cycle control mode
            self.get_logger().info("[MODE] Cycling control mode")
            self.current_key = None
        
        # Handle ESC separately (immediate action)
        if key == '\x1b':  # ESC
            self.get_logger().info("\nESC pressed - shutting down...")
            if self.is_tty and self.settings:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            rclpy.shutdown()
            return
        
        # Publish Joy message
        self.joy_pub.publish(joy_msg)
    
    def __del__(self):
        """Restore terminal settings on exit."""
        if self.is_tty and self.settings:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            except:
                pass


def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = DualArmKeyboardJoyPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Restore terminal
        if node and hasattr(node, 'is_tty') and node.is_tty and \
           hasattr(node, 'settings') and node.settings:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
            except:
                pass
        
        if node:
            node.destroy_node()
        
        # Only shutdown if not already shut down
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
