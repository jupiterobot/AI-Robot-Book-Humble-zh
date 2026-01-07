import sys
from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler
from math import pi, cos, sin


class PlanetBroadcaster(Node):

    def __init__(self, args):
        self.frame_id = 'planet'
        self.radius = 3.0  # [m]
        self.revolution_period = 16  # [s]
        timer_period = 0.1  # [s]
        argc = len(args)
        if argc >= 2:
            self.frame_id = args[1]
        if argc >= 3:
            self.radius = float(args[2])
        if argc >= 4:
            self.revolution_period = float(args[3])
        if argc >= 5:
            timer_period = float(args[4])
        super().__init__(self.frame_id)
        self.broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.start_time = self.get_clock().now()

    def timer_callback(self):
        transform_stamped = TransformStamped()
        now = self.get_clock().now()
        dt = (now - self.start_time).nanoseconds * 1e-9
        w = 2*pi/self.revolution_period
        theta = w * dt
        transform_stamped.header.stamp = now.to_msg()
        transform_stamped.header.frame_id = 'sun'
        transform_stamped.child_frame_id = self.frame_id
        transform_stamped.transform.translation.x = self.radius * cos(theta)
        transform_stamped.transform.translation.y = self.radius * sin(theta)
        transform_stamped.transform.translation.z = 0.0
        ratio = 2.0
        q = quaternion_from_euler(0, 0, ratio*theta)
        transform_stamped.transform.rotation.x = q[0]
        transform_stamped.transform.rotation.y = q[1]
        transform_stamped.transform.rotation.z = q[2]
        transform_stamped.transform.rotation.w = q[3]
        self.broadcaster.sendTransform(transform_stamped)


def main():
    rclpy.init()
    node = PlanetBroadcaster(remove_ros_args(args=sys.argv))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
