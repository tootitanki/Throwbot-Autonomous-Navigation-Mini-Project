"""Scripted moving obstacle: ping-pong along Y at constant speed, driven by SIM time.
Publishes std_msgs/Float64 velocity to /obstacle/cmd_vel (bridged to the Gazebo joint controller).
The obstacle starts parked at y=-3. It is released when the robot has driven trigger_odom_x metres, after a delay chosen so
that it reaches the robot's route (y=0) about `nominal_arrival` s after the trigger, plus a seeded random offset
(uniform +-jitter s). The same seed gives the same schedule for every sensor, so conditions are paired."""
import random
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry


class Mover(Node):
    def __init__(self):
        super().__init__('obstacle_mover')
        self.declare_parameter('speed', 0.5)
        self.declare_parameter('travel', 6.0)
        self.declare_parameter('trigger_odom_x', -4.0)  # odom is now the Gazebo world frame (robot spawns at x=-4.5): 0.5 m driven
        self.declare_parameter('nominal_arrival', 11.0)
        self.declare_parameter('jitter', 1.0)
        self.declare_parameter('seed', 0)
        s = self.get_parameter('speed').value
        rng = random.Random(self.get_parameter('seed').value)
        off = rng.uniform(-1, 1) * self.get_parameter('jitter').value
        self.delay = max(0.0, self.get_parameter('nominal_arrival').value - 3.0 / s + off)
        self.get_logger().info(f'speed={s} m/s seed={self.get_parameter("seed").value} start delay after trigger={self.delay:.2f} s')
        self.triggered = False
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.pub = self.create_publisher(Float64, '/obstacle/cmd_vel', 10)
        self.t0 = None
        self.create_timer(0.05, self.tick)

    def on_odom(self, m):
        if m.pose.pose.position.x > self.get_parameter('trigger_odom_x').value:
            self.triggered = True

    def tick(self):
        if not self.triggered:
            self.pub.publish(Float64(data=0.0))
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now == 0.0:
            return
        if self.t0 is None:
            self.t0 = now
        s = self.get_parameter('speed').value
        leg = self.get_parameter('travel').value / s
        t = now - self.t0 - self.delay
        v = 0.0
        if t >= 0:
            v = s if int(t // leg) % 2 == 0 else -s
        self.pub.publish(Float64(data=v))


def main():
    rclpy.init()
    rclpy.spin(Mover())
