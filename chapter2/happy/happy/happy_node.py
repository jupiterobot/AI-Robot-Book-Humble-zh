import rclpy  # 1. 导入 ROS2 Python 模块
from rclpy.node import Node  # 从 rclpy.node 模块导入 Node 类


class HappyNode(Node):  # HappyNode 类
    def __init__(self):
        print('正在创建节点')
        super().__init__('happy_node')  # 调用基类构造函数
        self.get_logger().info('快乐世界')  # 4. 节点的处理

 
def main():  # 主函数
    print('程序开始')
    rclpy.init()               # 2. 初始化
    node = HappyNode()         # 3. 创建节点
    node.destroy_node()        # 5. 销毁节点    
    rclpy.shutdown()           # 6. 关闭处理
    print('程序结束')