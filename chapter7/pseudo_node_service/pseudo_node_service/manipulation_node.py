from airobot_interfaces.srv import StringCommand

import rclpy
from rclpy.node import Node

import random
from time import sleep


class ManipulationServer(Node):

    def __init__(self):
        super().__init__('manipulation_server')
        self.srv = self.create_service(
            StringCommand, 'manipulation/command', self.command_callback)

    def command_callback(self, request, response):
        sleep(1)

        prob = random.random()
        self.get_logger().info('正在抓取目标物体')

        if 0.7 > prob:
            self.get_logger().info('目标物体抓取成功')
            response.answer = 'reached'
        else:
            self.get_logger().info('目标物体抓取失败')
            response.answer = 'failed'

        return response


def main(args=None):
    rclpy.init(args=args)

    manipulation_server = ManipulationServer()

    rclpy.spin(manipulation_server)

    rclpy.shutdown()