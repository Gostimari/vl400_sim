#!/usr/bin/env python3
"""Port of the ROS 1 pub_GPS_XY_trator.py.

Averages the last 40 GPS fixes, converts the position relative to a
fixed origin into local XY metres and publishes it as a NavSatFix on
'XY_coordinates' (longitude = -east, latitude = north), matching the
original node's output convention.
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

EARTH_RADIUS_M = 6371000.0
ORIGIN_LAT = 49.9
ORIGIN_LON = 8.9


def haversine_xy(point1, point2):
    """Return [east, north] offset in metres from point1 to point2.

    Replaces haversine_tt from the ROS 1 fun_cmd_vel module
    (small-angle local tangent plane approximation).
    """
    lat1 = math.radians(point1[0])
    lat2 = math.radians(point2[0])
    dlat = lat2 - lat1
    dlon = math.radians(point2[1]) - math.radians(point1[1])
    north = EARTH_RADIUS_M * dlat
    east = EARTH_RADIUS_M * dlon * math.cos((lat1 + lat2) / 2.0)
    return [east, north]


class Buff:
    def __init__(self):
        size_buff = 40
        self.x = [None] * size_buff
        self.y = [None] * size_buff
        self.avX = None
        self.avY = None

    def aver(self):
        if not (None in self.x and None in self.y):
            self.avX = sum(self.x) / len(self.x)
            self.avY = sum(self.y) / len(self.y)
            return [self.avX, self.avY]
        return [None, None]

    def add(self, vector):
        self.x.pop(0)
        self.x.append(vector.latitude)
        self.y.pop(0)
        self.y.append(vector.longitude)


class GpsXY(Node):

    def __init__(self):
        super().__init__('pub_XY_as_Coordinates')
        self.pub = self.create_publisher(NavSatFix, 'XY_coordinates', 10)
        self.sub = self.create_subscription(
            NavSatFix, '/fix', self.pos_act_callback, 10)
        self.actual = NavSatFix()
        self.buff = Buff()
        # 50 Hz, like the original loop
        self.timer = self.create_timer(0.02, self.tick)

    def pos_act_callback(self, msg):
        self.actual = msg

    def tick(self):
        xy_pub = NavSatFix()
        origem = [ORIGIN_LAT, ORIGIN_LON]
        self.buff.add(self.actual)
        media_actual = self.buff.aver()
        if not (None in media_actual):
            pos_xy = haversine_xy(origem, media_actual)
            xy_pub.longitude = -round(pos_xy[0], 4)
            xy_pub.latitude = round(pos_xy[1], 4)
            self.pub.publish(xy_pub)


def main(args=None):
    rclpy.init(args=args)
    node = GpsXY()
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
