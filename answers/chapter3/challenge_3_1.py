from threading import Thread, Event
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from airobot_interfaces.action import StringCommand

"""
挑战 3.1 的参考答案

改进了回声（鹦鹉学舌）程序
【所需程序】
・challenge_3_1.py              对 speech_client.py 进行部分修改
・speech_recognition_server.py  无需修改
・speech_synthesis_server.py    无需修改

新程序需要在 setup.py 中追加注册。
将 speech_client.py 重命名为 challenge_3_1_1.py，
并按照与回声程序相同的顺序启动程序。

"""


class StringCommandActionClient:
    def __init__(self, node, name):
        self.name = name
        self.logger = node.get_logger()
        self.action_client = ActionClient(node, StringCommand, name)
        self.event = Event()

    def send_goal(self, command: str):
        self.logger.info(f'{self.name} 等待动作服务器...')
        self.action_client.wait_for_server()
        goal_msg = StringCommand.Goal()
        goal_msg.command = command
        self.logger.info(f'{self.name} 发送目标... 命令: \'{command}\'')
        self.event.clear()
        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)
        self.action_result = None
        self.event.wait(20.0)
        if self.action_result is None:
            self.logger.info(f'{self.name} 超时')
            return None
        else:
            result = self.action_result.result
            status = self.action_result.status
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.logger.info(f'{self.name} 结果: {result.answer}')
                self.goal_handle = None
                return result.answer
            else:
                self.logger.info(f'{self.name} 失败状态: {status}')
                return None

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.logger.info(f'{self.name} 目标被拒绝')
            return
        self.goal_handle = goal_handle
        self.logger.info(f'{self.name} 目标已被接受')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.action_result = future.result()
        self.event.set()


class SpeechClient(Node):
    def __init__(self):
        super().__init__('speech_client')
        self.get_logger().info('启动语音对话节点。')
        self.recognition_client = StringCommandActionClient(
            self, 'speech_recognition/command')
        self.synthesis_client = StringCommandActionClient(
            self, 'speech_synthesis/command')
        self.thread = Thread(target=self.run)
        self.thread.start()

        # 新增的变量
        self.objects = ['bottle', 'cup']
        self.places = ['kitchen', 'living']

    # 修改了函数内部的部分内容
    def run(self):
        self.running = True
        while self.running:
            self.synthesis_client.send_goal('I\'m ready.')
            self.get_logger().info('I\'m ready.')

            text = None
            while text is None:
                text = self.recognition_client.send_goal('')

            target_object, target_place = self.search_object_and_place(text)

            self.synthesis_client.send_goal(f'I will go to the {target_place} and grab a {target_object}')
            self.get_logger().info(f'I will go to the {target_place} and grab a {target_object}')

    # 新增的函数
    def search_object_and_place(self, text):

        self.get_logger().info(f'接收到的文本 "{text}"')

        target_object = None
        target_place = None

        for _object in self.objects:
            if _object in text:
                target_object = _object

        for _place in self.places:
            if _place in text:
                target_place = _place

        return target_object, target_place



def main():
    rclpy.init()
    node = SpeechClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.running = False
        pass

    rclpy.try_shutdown()