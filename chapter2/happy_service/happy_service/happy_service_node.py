import rclpy
from rclpy.node import Node
from happy_interfaces.srv import AddHappy  # 从服务文件导入 AddHappy 类型


class HappyService(Node):  # Happy 服务类
    def __init__(self):  # 构造函数
        super().__init__('happy_service')
        # 创建服务（服务类型，服务名称，回调函数）
        self.service = self.create_service(AddHappy, 'add_happy',
                                           self.add_happy_callback)

    def add_happy_callback(self, request, response):  # 回调函数
        response.happy_word = 'Happy ' + request.word
        self.get_logger().info(f"收到请求。单词: {request.word}")
        return response


def main():  # 主函数
    rclpy.init()
    happy_service = HappyService()
    try:
        rclpy.spin(happy_service)
    except KeyboardInterrupt:
        print("检测到 Ctrl+C。")
    finally:
        happy_service.destroy_node()
        rclpy.shutdown()
    rclpy.shutdown()