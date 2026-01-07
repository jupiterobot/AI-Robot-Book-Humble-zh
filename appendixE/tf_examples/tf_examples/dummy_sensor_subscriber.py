import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs  # 虽不直接使用，但对transform()是必需的


class DummySensorSubscriber(Node):

    def __init__(self):
        super().__init__('dummy_sensor_subscriber')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.subscription = self.create_subscription(
            PointStamped, 'point', self.subscriber_callback, 10)
        self.publisher = self.create_publisher(PointStamped, 'point2', 10)

    def subscriber_callback(self, msg):
        try:
            msg2 = self.tf_buffer.transform(msg, 'base_link')
        except TransformException as ex:
            self.get_logger().info(str(ex))
            return
        self.publisher.publish(msg2)


def main():
    rclpy.init()
    node = DummySensorSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
