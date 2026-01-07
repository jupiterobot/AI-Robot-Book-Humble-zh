import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters, SetParameters

class ParameterClient(Node):
    def __init__(self):
        super().__init__('happy_parameter_client_node')
        self.client = self.create_client(GetParameters, '/happy_parameter_server_node/get_parameters'.format_map(locals()))        
        while not self.client.wait_for_service(timeout_sec=1.0):  # 等待服务可用
            self.get_logger().info('服务不可用，正在等待中...')        
        self.request = GetParameters.Request()  # 创建请求实例      
            
    def send_request(self, names):   # 发送请求        
        self.request.names = names  # 为请求赋值参数名称列表
        self.future = self.client.call_async(self.request)  # 发起服务请求
    

def main(args=None):
    rclpy.init(args=args)
    node = ParameterClient()
    names = ['happy_param_int','happy_param_string']  # 指定要获取的参数名列表
    node.send_request(names)

    while rclpy.ok():        
        rclpy.spin_once(node)
        if node.future.done():              # 当服务处理完成时
            try:
                res = node.future.result()  # 获取服务返回的结果
            except Exception as e:          # 若发生错误
                node.get_logger().info(f"服务调用失败。{e}")
            else:                           # 成功时，在日志中显示参数名及其值
                node.get_logger().info(f"{names[0]}: {res.values[0].integer_value}")
                node.get_logger().info(f"{names[1]}: {res.values[1].string_value}")
                break   
    rclpy.shutdown()