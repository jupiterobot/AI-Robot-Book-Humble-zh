import rclpy                         # ROS2 的 Python 模块
from rclpy.node import Node          # 从 rclpy.node 模块导入 Node 类
from std_msgs.msg import String      # 从 std_msgs.msg 模块导入 String 类


class HappyPublisher(Node):  # 发布并显示 "Happy World" 的类（实际发布倒计时）
    def __init__(self):  # 构造函数
        super().__init__('happy_publisher_node')
        self.pub = self.create_publisher(String, 'topic', 10)   # 创建发布者
        self.timer = self.create_timer(1, self.timer_callback)  # 创建定时器
        self.i = 10

    def timer_callback(self):  # 回调函数
        msg = String()
        if self.i > 0:
            msg.data = f'快乐倒计时 {self.i}'
        elif self.i == 0:
            msg.data = f'发射！'
        else:
            msg.data = f'经过时间 {-self.i}'
        self.pub.publish(msg)
        self.get_logger().info(f'已发布: {msg.data}')
        self.i -= 1


def main(args=None):  # 主函数
    rclpy.init()
    node = HappyPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('检测到 Ctrl+C。')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()