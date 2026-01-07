import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

class ParameterServer(Node):  # ParameterServer 类
    def __init__(self):
        super().__init__('happy_parameter_server_node')
        # 声明参数并设置初始值
        self.declare_parameter('happy_param_int', Parameter.Type.INTEGER)  
        self.declare_parameter('happy_param_string', 'Happy World')  # 声明参数
        # 将参数值更新为新的整数值
        self.set_parameters([Parameter('happy_param_int', Parameter.Type.INTEGER, 7)]) 
        self.timer = self.create_timer(1, self.timer_callback)  # 创建定时器
    
    def timer_callback(self):
        param_int    = self.get_parameter('happy_param_int').value  # 获取参数值
        param_string = self.get_parameter('happy_param_string').value  # 获取参数值
        self.get_logger().info(f'happy_param_int: {param_int}')  # 显示参数值
        self.get_logger().info(f'happy_param_string: {param_string}')  # 显示参数值

# 主函数
def main(args=None):
    rclpy.init(args=args)  # 初始化 ROS 2 的 Python 客户端库
    node = ParameterServer()  # 创建 ParameterServer 类的实例
    try:
        rclpy.spin(node)  # 运行节点，使其能够处理回调函数
    except KeyboardInterrupt:
        pass  # 捕获键盘中断（Ctrl+C）并正常退出
    rclpy.shutdown()  # 执行 ROS 2 的关闭流程

if __name__ == '__main__':
    main()