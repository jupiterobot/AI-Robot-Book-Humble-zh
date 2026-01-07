import rclpy
from rclpy.node import Node
from airobot_interfaces.srv import StringCommand


class BringmeClient(Node):
    def __init__(self):  # 构造函数
        super().__init__('bringme_client_node')
        self.client = self.create_client(StringCommand, 'command')  # 创建客户端
        # 等待服务可用
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('服务不可用，正在等待中...')
        self.request = StringCommand.Request()  # 创建请求实例

    def send_request(self, order):  # 发送请求的方法
        self.request.command = order  # 为请求赋值
        self.future = self.client.call_async(self.request)  # 发起服务请求


def main():
    rclpy.init()
    bringme_client = BringmeClient()
    order = input('要拿什么过来：')  # ← 用户提示已翻译
    bringme_client.send_request(order)

    while rclpy.ok():
        rclpy.spin_once(bringme_client)  # 单次运行节点，处理回调
        if bringme_client.future.done():  # 检查服务是否已完成
            try:
                response = bringme_client.future.result()  # 获取服务结果
            except Exception as e:
                bringme_client.get_logger().info(f'服务调用失败：{e}')
            else:
                bringme_client.get_logger().info(  # 显示结果（全中文描述）
                    f'\n请求内容: {bringme_client.request.command} → 机器人回复: {response.answer}')
                break
    bringme_client.destroy_node()
    rclpy.shutdown()