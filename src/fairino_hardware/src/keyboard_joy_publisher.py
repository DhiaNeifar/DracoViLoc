#!/usr/bin/env python3
"""
Keyboard Joy Publisher for Fairino Dual Robot Control
Converts keyboard input to Joy messages compatible with joystick_controller.cpp

Keyboard Mapping (WASD Gaming Style):
  Translation:
    a/d     → X-axis (left/right)
    w/s     → Y-axis (forward/back)
    q/e     → Z-axis (down/up)
  
  Rotation:
    j/l     → A rotation (roll)
    i/k     → B rotation (pitch)
    u/o     → C rotation (yaw)
  
  Control:
    c       → Switch robot (Button 1)
    r       → Reset errors (Button 3)
    ESC     → Quit

Author: GitHub Copilot
Date: March 13, 2026
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import sys
import select
import termios
import tty


class KeyboardJoyPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_joy_publisher')
        
        # Publisher to /joy topic (same as joystick)
        self.joy_pub = self.create_publisher(Joy, '/joy', 10)
        
        # Publish rate: 50Hz to match joystick controller's control rate
        self.timer = self.create_timer(0.1, self.publish_joy)
        
        # Current key pressed and hold counter
        self.current_key = None
        self.key_hold_cycles = 0
        self.KEY_HOLD_DURATION = 10  # Hold key for 10 cycles (200ms) after detection
        
        # Check if stdin is a terminal (TTY)
        self.is_tty = sys.stdin.isatty()
        
        # Terminal settings for raw keyboard input (only if TTY)
        if self.is_tty:
            self.settings = termios.tcgetattr(sys.stdin)
            self.get_logger().info("=" * 70)
            self.get_logger().info("  Keyboard Control - DUAL ARM ROBOT SYSTEM")
            self.get_logger().info("=" * 70)
            self.get_logger().info("")
            self.get_logger().info("🤖 ROBOT SWITCHING:")
            self.get_logger().info("  c       - Switch between Robot 1 (left) & Robot 2 (right)")
            self.get_logger().info("  r       - Reset errors on active robot")
            self.get_logger().info("")
            self.get_logger().info("📍 TRANSLATION (currently active robot):")
            self.get_logger().info("  a/d     - X-axis (left/right)")
            self.get_logger().info("  w/s     - Y-axis (forward/backward)")
            self.get_logger().info("  q/e     - Z-axis (down/up)")
            self.get_logger().info("")
            self.get_logger().info("🔄 ROTATION (currently active robot):")
            self.get_logger().info("  j/l     - A rotation (roll)")
            self.get_logger().info("  i/k     - B rotation (pitch)")
            self.get_logger().info("  u/o     - C rotation (yaw)")
            self.get_logger().info("")
            self.get_logger().info("⚙️  CONTROL:")
            self.get_logger().info("  ESC     - Quit")
            self.get_logger().info("=" * 70)
            self.get_logger().info("✓ Starting with Robot 1 active (192.168.58.2)")
            self.get_logger().info("  Hold key to move, release to stop. Press 'c' to switch robots.")
            self.get_logger().info("=" * 70)
        else:
            self.settings = None
            self.get_logger().warn("Keyboard input disabled - stdin is not a TTY")
            self.get_logger().warn("To use keyboard control, run directly in terminal:")
            self.get_logger().warn("  ros2 run fairino_hardware keyboard_joy_publisher.py")
    
    def get_key(self, timeout=0.1):
        """Get single keypress without blocking."""
        if not self.is_tty:
            return ''  # No keyboard input when not in terminal
        
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
        elif active_key == 'c':
            joy_msg.buttons[1] = 1  # X button (switch robot)
            self.get_logger().info("🔄 [SWITCH] Press 'c' to toggle between robots")
            self.current_key = None  # Clear immediately for one-shot action
        elif active_key == 'r':
            joy_msg.buttons[3] = 1  # Triangle button (reset errors)
            self.get_logger().info("🔧 [RESET] Resetting errors on active robot")
            self.current_key = None  # Clear immediately for one-shot action
        
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
    
    try:
        node = KeyboardJoyPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Restore terminal
        if hasattr(node, 'is_tty') and node.is_tty and hasattr(node, 'settings') and node.settings:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
            except:
                pass
        
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
