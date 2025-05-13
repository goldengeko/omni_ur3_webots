#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog

import sys
import tty
import termios
import threading
import signal

# Constants
TWIST_TOPIC = "/servo_node/delta_twist_cmds"
JOINT_TOPIC = "/servo_node/delta_joint_cmds"
EEF_FRAME_ID = "tool0"
BASE_FRAME_ID = "base_link"

KEY_BINDINGS = {
    "\x1b[A": ("twist", "x", 1.0),  # UP arrow
    "\x1b[B": ("twist", "x", -1.0),  # DOWN arrow
    "\x1b[C": ("twist", "y", 1.0),  # RIGHT arrow
    "\x1b[D": ("twist", "y", -1.0),  # LEFT arrow
    ".": ("twist", "z", -1.0),
    ";": ("twist", "z", 1.0),
    "w": ("twist_angular", "x", 0.5),
    "s": ("twist_angular", "x", -0.5),
    "a": ("twist_angular", "y", -0.5),
    "d": ("twist_angular", "y", 0.5),
    "z": ("twist_angular", "z", -0.5),
    "x": ("twist_angular", "z", 0.5),
    "1": ("joint", "shoulder_pan_joint"),
    "2": ("joint", "shoulder_lift_joint"),
    "3": ("joint", "elbow_joint"),
    "4": ("joint", "wrist_1_joint"),
    "5": ("joint", "wrist_2_joint"),
    "6": ("joint", "wrist_3_joint"),
    "b": ("frame", BASE_FRAME_ID),
    "e": ("frame", EEF_FRAME_ID),
    "r": ("reverse", None),
    "q": ("quit", None),
}


class ServoKeyboardInput(Node):
    def __init__(self):
        super().__init__("servo_keyboard_input")
        self.twist_pub = self.create_publisher(TwistStamped, TWIST_TOPIC, 10)
        self.joint_pub = self.create_publisher(JointJog, JOINT_TOPIC, 10)

        self.frame_to_publish = BASE_FRAME_ID
        self.joint_vel_cmd = 1.0

        self.running = True
        signal.signal(signal.SIGINT, self.shutdown_handler)

    def shutdown_handler(self, signum, frame):
        self.running = False
        rclpy.shutdown()

    def read_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
            if key == "\x1b":
                key += sys.stdin.read(2)
            return key
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def key_loop(self):
        print(
            """
Reading from keyboard:

Use arrow keys and the '.' and ';' keys to Cartesian jog.
Use 'w', 'a', 's', 'd', 'z', 'x' to angular jog.
Use 1|2|3|4|5|6 keys to joint jog. 'r' to reverse the direction of jogging.
Press 'b' to switch to the base frame, 'e' to switch to the end-effector frame.
Press 'q' to quit.
"""
        )
        while self.running:
            key = self.read_key()
            if key in KEY_BINDINGS:
                binding = KEY_BINDINGS[key]
                action = binding[0]

                if len(binding) > 1:
                    value = binding[1]
                else:
                    value = None

                if action == "twist":
                    msg = TwistStamped()
                    msg.twist.linear.x = 0.0
                    msg.twist.linear.y = 0.0
                    msg.twist.linear.z = 0.0
                    if value in ["x", "y", "z"]:
                        setattr(msg.twist.linear, value, binding[2])
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = self.frame_to_publish
                    self.twist_pub.publish(msg)

                elif action == "twist_angular":
                    msg = TwistStamped()
                    msg.twist.angular.x = 0.0
                    msg.twist.angular.y = 0.0
                    msg.twist.angular.z = 0.0
                    if value in ["x", "y", "z"]:
                        setattr(msg.twist.angular, value, binding[2])
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = self.frame_to_publish
                    self.twist_pub.publish(msg)

                elif action == "joint":
                    msg = JointJog()
                    msg.joint_names = [value]
                    msg.velocities = [self.joint_vel_cmd]
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = BASE_FRAME_ID
                    self.joint_pub.publish(msg)

                elif action == "frame":
                    self.frame_to_publish = value
                    self.get_logger().info(f"Switched frame to {value}")

                elif action == "reverse":
                    self.joint_vel_cmd *= -1
                    self.get_logger().info(
                        f"Reversed joint velocity: {self.joint_vel_cmd}"
                    )

                elif action == "quit":
                    self.get_logger().info("Exiting keyboard teleop.")
                    break
            else:
                self.get_logger().debug(f"Unhandled key: {repr(key)}")


def main(args=None):
    rclpy.init(args=args)
    node = ServoKeyboardInput()

    try:
        node.key_loop()
    except Exception as e:
        node.get_logger().error(f"Error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
