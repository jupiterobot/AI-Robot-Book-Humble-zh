import sys
import rclpy
from rclpy.node import Node
from happy_interfaces.srv import AddHappy

class HappyClient(Node):
    def __init__(self):
        super().__init__('happy_client_node')
        # 创建客户端
        self.client = self.create_client(AddHappy, 'add_happy')
        # 等待服务可用
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('服务不可用，正在等待中...')
        # 创建请求实例
        self.request = AddHappy.Request()
            
    def send_request(self):
        # 为请求赋值
        self.request.word = str(sys.argv[1])
        # 发送服务请求
        self.future = self.client.call_async(self.request)

def main(args=None):
    rclpy.init(args=args)
    happy_client = HappyClient()
    happy_client.send_request()
    while rclpy.ok():
        rclpy.spin_once(happy_client)
        if happy_client.future.done():  # 当服务处理完成时
            try:
                response = happy_client.future.result()  # 将服务结果赋值给 response
            except Exception as e:
                happy_client.get_logger().info(f"服务调用失败。{e}")
            else:
                # 显示结果
                happy_client.get_logger().info(f"请求: {happy_client.request.word} -> 响应: {response.happy_word}")
                break        
    happy_client.destroy_node()
    rclpy.shutdown()