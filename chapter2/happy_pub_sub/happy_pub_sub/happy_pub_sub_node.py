import sys
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String, Int32


class HappyPubSub(Node):
    def __init__(self):
        super().__init__('happy_pub_sub')
        self.pub = self.create_publisher(String, 'happy_msg', 10)
        self.sub = self.create_subscription(Int32, 'number', self.callback, 10)
        self.happy_actions = ({
            1: '对他人做出友善的行为。',
            2: '与他人建立联系。',
            3: '为了健康而运动。',
            4: '进行正念练习。',
            5: '挑战新事物。',
            6: '设定目标，并迈出第一步。',
            7: '培养心理韧性（恢复力）。',
            8: '关注事物积极的一面。',
            9: '接纳人与人之间的差异。',
            10: '大家携手让世界变得更美好。'})

    def callback(self, sub_msg):
        pub_msg = String()
        self.get_logger().info(f'已订阅: {sub_msg.data}')
        pub_msg.data = self.happy_actions[sub_msg.data % 10 + 1]
        self.pub.publish(pub_msg)
        self.get_logger().info(f'已发布: {pub_msg.data}')


def main():
    rclpy.init()
    node = HappyPubSub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()