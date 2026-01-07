import rclpy
from rclpy.node import Node
from airobot_interfaces.srv import StringCommand

import random
from time import sleep


class VoiceServer(Node):

    def __init__(self):
        super().__init__('voice_server')
        self.srv = self.create_service(
            StringCommand, 'voice/command', self.command_callback)

    def command_callback(self, request, response):
        sleep(1)

        prob = random.random()
        self.get_logger().info(f'开始进行语音识别')

        if 0.5 > prob:
            self.get_logger().info('语音识别成功')
            response.answer = 'bring me a cup from the kitchen'
        else:
            self.get_logger().info('语音识别失败')
            response.answer = ''

        return response


def main(args=None):
    rclpy.init(args=args)

    voice_server = VoiceServer()

    rclpy.spin(voice_server)

    rclpy.shutdown()