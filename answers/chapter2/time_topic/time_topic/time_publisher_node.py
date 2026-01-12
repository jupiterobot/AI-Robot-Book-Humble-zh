import rclpy                         # ROS2 的 Python 模块
from rclpy.node import Node          # 从 rclpy.node 模块导入 Node 类
from std_msgs.msg import String      # 从 std_msgs.msg 模块导入 String 类
import datetime                      # 导入用于获取日期时间的库
import pytz                          # 为支持中国时间而导入时区处理库


class TimePublisher(Node):  # 每秒发布中国标准时间的类
    def __init__(self):     # 构造函数
        super().__init__('time_publisher_node')
        self.pub = self.create_publisher(String, 'topic', 10)   # 创建发布者
        self.timer = self.create_timer(1, self.timer_callback)  # 创建定时器
        self.cst = pytz.timezone('Asia/Shanghai')               # 设置中国标准时间（CST）时区
 
    def timer_callback(self):  # 回调函数
        msg = String()
        cst_time = datetime.datetime.now(self.cst).strftime('%Y-%m-%d %H:%M:%S')  # 获取当前中国标准时间并转换为字符串
        msg.data = cst_time
        self.pub.publish(msg)
        self.get_logger().info(f'已发布: {msg.data}')


def main(args=None):  # 主函数
    rclpy.init()
    node = TimePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('检测到 Ctrl+C。')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()