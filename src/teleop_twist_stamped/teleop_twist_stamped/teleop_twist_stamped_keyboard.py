#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from rclpy.qos import QoSProfile

import sys
import termios
import tty
import select
import threading

class TwistStampedPublisher(Node):
    def __init__(self):
        super().__init__('TwistStampedPublisher')
        self.publisher = self.create_publisher(
            TwistStamped,
            '/diff_drive_base_controller/cmd_vel',
            QoSProfile(depth=10)
        )

        # Initial speeds
        self.speed = 0.2
        self.turn = 0.3
        self.x = 0
        self.z = 0

        self.settings = termios.tcgetattr(sys.stdin)

        # Start the publishing timer
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

        # Start keyboard thread
        thread = threading.Thread(target=self.keyboard_listener)
        thread.daemon = True
        thread.start()

        self.print_help()

    def print_help(self):
        self.get_logger().info(
            """
                i: forward || , (comma): backward || j: rotate left || l: rotate right || k: stop || q/z: increase/decrease linear speed || e/c: increase/decrease angular speed
            """
        )

    def keyboard_listener(self):
        try:
            while True:
                key = self.get_key()
                if key == 'i':
                    self.x = 1
                    self.z = 0
                elif key == ',':
                    self.x = -1
                    self.z = 0
                elif key == 'j':
                    self.x = 0
                    self.z = 1
                elif key == 'l':
                    self.x = 0
                    self.z = -1
                elif key == 'k':
                    self.x = 0
                    self.z = 0
                elif key == 'q':
                    self.speed *= 1.1
                elif key == 'z':
                    self.speed *= 0.9
                elif key == 'e':
                    self.turn *= 1.1
                elif key == 'c':
                    self.turn *= 0.9
                elif key == '\x03':  # CTRL-C
                    break

                self.get_logger().info(f"Speed: {self.speed:.2f}, Turn: {self.turn:.2f}")
        except Exception as e:
            self.get_logger().error(f"Keyboard error: {e}")
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def timer_callback(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = self.x * self.speed
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = self.z * self.turn

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TwistStampedPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
