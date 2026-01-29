#!/usr/bin/env python3
"""
Keyboard Teleoperation Node for Perseus Arm.
"""

import sys
import os
import termios
import tty
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from moveit_msgs.srv import ServoCommandType
from typing import List, Optional

ARROW_PREFIX = '\x1b'
KEY_UP = '\x1b[A'
KEY_DOWN = '\x1b[B'
KEY_RIGHT = '\x1b[C'
KEY_LEFT = '\x1b[D'


class KeyboardTeleop(Node):

    def __init__(self) -> None:
        super().__init__('keyboard_teleop')
        
        # Publisher
        self.twist_pub = self.create_publisher(
            TwistStamped, 
            '/servo_node/delta_twist_cmds', 
            10
        )
        self.joint_pub = self.create_publisher(
            JointJog, 
            '/servo_node/delta_joint_cmds', 
            10
        )
        
        # Service client
        self.switch_client = self.create_client(
            ServoCommandType, 
            '/servo_node/switch_command_type'
        )
        
        # config
        self.mode: str = 'twist'
        self.frame_id: str = 'plate'
        self.selected_joint: int = 2
        
        self.joint_names: List[str] = [
            'shoulder_pan', 
            'shoulder_tilt', 
            'elbow', 
            'wrist_pitch', 
            'wrist_roll'
        ]
        
        # Control Parameters
        self.linear_speed: float = 2.0
        self.joint_speed: float = 1.0
        
        # State for non-blocking control
        self.active_twist = TwistStamped()
        self.active_joint = JointJog()
        self.last_key_time = 0.0
        self.key_timeout = 0.2 
        
        self.create_timer(0.02, self.control_loop)
        
        self.get_logger().info('Keyboard Teleop Initialised (High Performance Mode).')
        self.print_help()

    def control_loop(self):
        if time.time() - self.last_key_time > self.key_timeout:
            self.active_twist = TwistStamped()
            self.active_twist.header.frame_id = self.frame_id
            self.active_twist.header.stamp = self.get_clock().now().to_msg()
            
            self.active_joint = JointJog()
            self.active_joint.header.stamp = self.get_clock().now().to_msg()
            self.active_joint.velocities = [0.0]
        else:
            if self.mode == 'twist':
                self.active_twist.header.stamp = self.get_clock().now().to_msg()
                self.active_twist.header.frame_id = self.frame_id
                self.twist_pub.publish(self.active_twist)
            elif self.mode == 'joint':
                self.active_joint.header.stamp = self.get_clock().now().to_msg()
                self.joint_pub.publish(self.active_joint)

    def update_twist(self, lx=0.0, ly=0.0, lz=0.0):
        self.active_twist.twist.linear.x = lx
        self.active_twist.twist.linear.y = ly
        self.active_twist.twist.linear.z = lz
        self.last_key_time = time.time()

    def update_joint(self, velocity):
        self.active_joint.joint_names = [self.joint_names[self.selected_joint]]
        self.active_joint.velocities = [velocity]
        self.last_key_time = time.time()

    def print_help(self) -> None:
        print(f"\n=== Perseus Teleop | Mode: {self.mode.upper()} ===")
        
        status_line = f"Frame: {self.frame_id}" if self.mode == 'twist' else \
                      f"Joint: {self.joint_names[self.selected_joint]}"
        print(f"Status: {status_line}")
        print("---------------------------------------------------")
        print(" [t] Twist Mode   [j] Joint Mode")
        print(" [w] World Frame  [e] End-Effector Frame")
        print("---------------------------------------------------")
        
        if self.mode == 'twist':
            print(" Controls:")
            print("   ↑ / ↓        : Z-Axis (Up/Down)")
            print("   ← / →        : Y-Axis (Left/Right)")
            print("   n / m        : X-Axis (Forward/Backward)")
        else:
            print(" Controls:")
            print("   1 - 5        : Select Joint")
            print("   ↑ / ↓        : Jog Joint (+/-)")
            
        print("---------------------------------------------------")
        print(" [q] Quit")
        print("---------------------------------------------------\n")

    def switch_servo_mode(self, mode_type: str) -> None:
        if not self.switch_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Switch service not available - proceed with caution.')
            
        req = ServoCommandType.Request()
        req.command_type = 0 if mode_type == 'joint' else 1
        self.switch_client.call_async(req)
        
        self.mode = mode_type
        self.last_key_time = 0.0

    def _read_key(self) -> str:
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch_bytes = os.read(fd, 1)
            ch = ch_bytes.decode('utf-8', errors='ignore')
            if ch == ARROW_PREFIX:
                seq_bytes = os.read(fd, 2)
                ch += seq_bytes.decode('utf-8', errors='ignore')
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def run(self) -> None:
        self.switch_servo_mode('twist')
        
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        
        try:
            tty.setraw(fd)
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.01)
                
                import select
                rlist, _, _ = select.select([sys.stdin], [], [], 0.01)
                if rlist:
                    ch_bytes = os.read(fd, 1)
                    key = ch_bytes.decode('utf-8', errors='ignore')
                    if key == ARROW_PREFIX:
                        seq_bytes = os.read(fd, 2)
                        key += seq_bytes.decode('utf-8', errors='ignore')
                        
                    # Process Key
                    if key == 'q' or key == '\x03': break
                    self.process_key_event(key)
                    
        except KeyboardInterrupt:
            pass
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            self.get_logger().info('Teleop Node Stopped.')

    def process_key_event(self, key):
        # Global
        if key == 't': 
            self.switch_servo_mode('twist'); print("\r\n[Twist]\r\n"); return
        if key == 'j': 
            self.switch_servo_mode('joint'); print("\r\n[Joint]\r\n"); return
        if key == 'w': 
            self.frame_id = 'plate'; print("\r\n[World]\r\n"); return
        if key == 'e': 
            self.frame_id = 'forearm_tip'; print("\r\n[EE]\r\n"); return

        # Motion
        if self.mode == 'twist':
            if key == KEY_UP: self.update_twist(lz=self.linear_speed)
            if key == KEY_DOWN: self.update_twist(lz=-self.linear_speed)
            if key == KEY_LEFT: self.update_twist(ly=self.linear_speed)
            if key == KEY_RIGHT: self.update_twist(ly=-self.linear_speed)
            if key == 'n': self.update_twist(lx=self.linear_speed)
            if key == 'm': self.update_twist(lx=-self.linear_speed)
        else:
            if key.isdigit() and 1 <= int(key) <= 5:
                self.selected_joint = int(key) - 1
                print(f"\r\n[Joint: {self.joint_names[self.selected_joint]}]\r\n")
            if key == KEY_UP: self.update_joint(self.joint_speed)
            if key == KEY_DOWN: self.update_joint(-self.joint_speed)


def main() -> None:
    rclpy.init()
    node = KeyboardTeleop()
    
    try:
        node.run()
    except Exception as e:
        node.get_logger().error(f'Runtime Error: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
