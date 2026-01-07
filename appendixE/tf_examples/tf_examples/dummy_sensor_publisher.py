from math import pi, cos, sin
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from rclpy.time import Duration


class DummySensorPublisher(Node):

    def __init__(self):
        super().__init__('dummy_sensor_publisher')
        self.publisher = self.create_publisher(PointStamped, 'point', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.start_time = self.get_clock().now()

    def timer_callback(self):
        msg = PointStamped()
        now = self.get_clock().now()
        msg.header.stamp = (now - Duration(seconds=0.1)).to_msg()  # 提前0.1[s]
        msg.header.frame_id = 'dummy_sensor'
        radius = 0.5
        distance = 0.5
        period = 10
        dt = (now - self.start_time).nanoseconds * 1e-9
        theta = 2 * pi / period * dt
        msg.point.x = radius * cos(theta)
        msg.point.y = radius * sin(theta)
        msg.point.z = distance
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = DummySensorPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
