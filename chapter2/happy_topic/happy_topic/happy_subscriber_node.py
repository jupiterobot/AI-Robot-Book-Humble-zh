import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# 一个简单的类，用于订阅 String 类型消息并在终端中显示
class HappySubscriber(Node):
    def __init__(self):  # 构造函数
        super().__init__('happy_subscriber_node')
        # 创建订阅者
        self.sub = self.create_subscription(String,
                                            'topic', self.callback, 10)

    def callback(self, msg):  # 回调函数
        self.get_logger().info(f'已订阅: {msg.data}')


def main(args=None):  # 主函数
    rclpy.init()
    node = HappySubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('检测到 Ctrl+C。')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()