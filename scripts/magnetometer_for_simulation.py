#!/usr/bin/env python3
"""Port of the ROS 1 magnetometer_for_simulation.py.

Subscribes to the magnetometer (bridged from gz-sim as
sensor_msgs/MagneticField), low-pass filters the x/y components and
publishes the compass heading in degrees on 'heading'.
"""
import math
from collections import deque

import numpy as np
import scipy.signal

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField
from std_msgs.msg import Float32


class LiveFilter:
    """Base class for live filters."""

    def process(self, x):
        if np.isnan(x):
            return x
        return self._process(x)

    def __call__(self, x):
        return self.process(x)

    def _process(self, x):
        raise NotImplementedError('Derived class must implement _process')


class LiveLFilter(LiveFilter):

    def __init__(self, b, a):
        self.a = a
        self.b = b
        self._xs = deque([0] * len(b), maxlen=len(b))
        self._ys = deque([0] * (len(a) - 1), maxlen=len(a) - 1)

    def _process(self, x):
        self._xs.appendleft(x)
        y = np.dot(self.b, self._xs) - np.dot(self.a[1:], self._ys)
        y = y / self.a[0]
        self._ys.appendleft(y)
        return y


class Magneto(Node):

    def __init__(self):
        super().__init__('magnetomer_node')
        self.vector_mag_x = 1.0
        self.vector_mag_y = 1.0
        self.vector_mag_z = 1.0
        fs = 13.5  # sampling rate, Hz
        b, a = scipy.signal.iirfilter(
            4, Wn=0.75, fs=fs, btype='low', ftype='butter')
        self.live_lfilter_x = LiveLFilter(b, a)
        self.live_lfilter_y = LiveLFilter(b, a)
        self.pub = self.create_publisher(Float32, 'heading', 10)
        self.sub = self.create_subscription(
            MagneticField, '/mag', self.callback_mag_data, 10)
        self.timer = self.create_timer(0.05, self.tick)  # 20 Hz

    def callback_mag_data(self, msg):
        self.vector_mag_x = msg.magnetic_field.x
        self.vector_mag_y = -msg.magnetic_field.y
        self.vector_mag_z = msg.magnetic_field.z

    def tick(self):
        x_live_lfilter = self.live_lfilter_x(self.vector_mag_x)
        y_live_lfilter = self.live_lfilter_y(self.vector_mag_y)
        heading = math.degrees(
            math.atan2(y_live_lfilter, x_live_lfilter))
        self.pub.publish(Float32(data=heading))


def main(args=None):
    rclpy.init(args=args)
    node = Magneto()
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
