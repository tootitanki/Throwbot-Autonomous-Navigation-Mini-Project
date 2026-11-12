"""Ultrasonic APPROXIMATION: collapses the narrow-cone ray scan to ONE range per update (the nearest echo in the cone),
published as sensor_msgs/Range (ULTRASOUND). Gazebo Fortress has no native sonar sensor (see README)."""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range


class R(Node):
    def __init__(self):
        super().__init__('range_from_scan')
        self.pub = self.create_publisher(Range, '/sensor/range', 10)
        self.create_subscription(LaserScan, '/sensor/scan', self.cb, 10)

    def cb(self, m):
        v = [r for r in m.ranges if math.isfinite(r) and m.range_min <= r < m.range_max]
        out = Range()
        out.header = m.header
        out.radiation_type = Range.ULTRASOUND
        out.field_of_view = m.angle_max - m.angle_min
        out.min_range = float(m.range_min)
        out.max_range = float(m.range_max)
        out.range = float(min(v)) if v else float(m.range_max)   # no echo -> max range (clears cells)
        self.pub.publish(out)


def main():
    rclpy.init(); rclpy.spin(R())
