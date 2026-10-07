#!/usr/bin/env python3
"""Port of the ROS 1 fake_state_publisher.py.

Publishes False on 'stop' at 20 Hz and, once every ~1000 ticks,
a random True/False, mimicking the original node.
"""
import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class FakeState(Node):

    def __init__(self):
        super().__init__('fake_state')
        self.pub = self.create_publisher(Bool, 'stop', 10)
        self.timer = self.create_timer(0.05, self.tick)  # 20 Hz
        self.count = 0

    def tick(self):
        msg = Bool()
        if self.count >= 1000:
            msg.data = random.choice([True, False])
            self.count = 0
        else:
            msg.data = False
            self.count += 1
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeState()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
