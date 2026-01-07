import rclpy
from rclpy.node import Node
from airobot_interfaces.srv import StringCommand

import random
from time import sleep


class VisionServer(Node):

    def __init__(self):
        super().__init__('vision_server')
        self.srv = self.create_service(
            StringCommand, 'vision/command', self.command_callback)

    def command_callback(self, request, response):

        prob = random.random()
        self.get_logger().info(f'开始寻找目标物体')

        sleep(1)

        if 0.5 > prob:
            self.get_logger().info('成功检测到目标物体')
            response.answer = 'detected'
        else:
            self.get_logger().info('未能检测到目标物体')
            response.answer = 'failed'

        return response


def main(args=None):
    rclpy.init(args=args)

    vision_server = VisionServer()

    rclpy.spin(vision_server)

    rclpy.shutdown()