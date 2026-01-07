import rclpy  # 1. 导入 ROS2 Python 模块
from rclpy.node import Node  # 从 rclpy.node 模块导入 Node 类


class HappyNode3(Node):  # HappyNode3 类
    def __init__(self):  # 构造函数
        print("正在创建节点")
        super().__init__('happy_node3')  # 调用基类构造函数
        self.timer = self.create_timer(1.0, self.timer_callback)  # 创建定时器

    def timer_callback(self):  # 回调函数
        self.get_logger().info('快乐世界３')  # 在终端显示


def main():  # 主函数
    print('程序开始')
    rclpy.init()                # 2. 初始化
    node = HappyNode3()         # 3. 创建节点
    try:                        # 异常处理，用于优雅退出       
        rclpy.spin(node)        # 4. 运行节点：反复调用回调函数
    except KeyboardInterrupt:
        print('检测到 Ctrl+C。')
    node.destroy_node()         # 5. 销毁节点
    rclpy.try_shutdown()        # 6. 关闭处理
    print('程序结束')