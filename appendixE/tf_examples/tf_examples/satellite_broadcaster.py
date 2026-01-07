from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class SatelliteBroadcaster(Node):

    def __init__(self):
        super().__init__('satellite_broadcaster')
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = 'planet'
        transform_stamped.child_frame_id = 'satellite'
        transform_stamped.transform.translation.x = 1.0
        transform_stamped.transform.translation.y = 0.0
        transform_stamped.transform.translation.z = 0.0
        transform_stamped.transform.rotation.x = 0.0
        transform_stamped.transform.rotation.y = 0.0
        transform_stamped.transform.rotation.z = 0.0
        transform_stamped.transform.rotation.w = 1.0
        broadcaster = StaticTransformBroadcaster(self)
        broadcaster.sendTransform(transform_stamped)


def main():
    rclpy.init()
    node = SatelliteBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
