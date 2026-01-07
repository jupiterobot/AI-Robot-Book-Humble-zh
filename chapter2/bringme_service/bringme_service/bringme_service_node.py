import time
import rclpy
from rclpy.node import Node
from airobot_interfaces.srv import StringCommand

class BringmeService(Node):  # 取物服务类
    def __init__(self):  # 构造函数
        super().__init__('bringme_service')
        # 创建服务（服务类型，服务名称，回调函数）
        self.service = self.create_service(StringCommand, 'command', self.callback)
        self.food = ['apple', 'banana', 'candy']   

    def callback(self, request, response):  # 回调函数
        time.sleep(5)
        item = request.command
        if item in self.food:
            response.answer = f'好的，这是 {item}。'
        else:
            response.answer = f'抱歉，找不到 {item}。'
        self.get_logger().info(f'响应: {response.answer}')
        return response


def main():  # 主函数
    rclpy.init()
    bringme_service = BringmeService()
    try:
        rclpy.spin(bringme_service)
    except KeyboardInterrupt:
        pass
    rclpy.try_shutdown()
    print('服务已结束')