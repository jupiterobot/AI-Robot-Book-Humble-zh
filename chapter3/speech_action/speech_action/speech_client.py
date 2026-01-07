from threading import Thread, Event
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from airobot_interfaces.action import StringCommand


class StringCommandActionClient:
    def __init__(self, node, name):
        self.name = name
        self.logger = node.get_logger()
        self.action_client = ActionClient(node, StringCommand, name)
        self.event = Event()

    def send_goal(self, command: str):
        self.logger.info(f'{self.name} 正在等待动作服务器...')
        self.action_client.wait_for_server()
        goal_msg = StringCommand.Goal()
        goal_msg.command = command
        self.logger.info(f'{self.name} 正在发送目标... 命令: \'{command}\'')
        self.event.clear()
        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)
        self.action_result = None
        self.event.wait(20.0)
        if self.action_result is None:
            self.logger.info(f'{self.name} 请求超时')
            return None
        else:
            result = self.action_result.result
            status = self.action_result.status
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.logger.info(f'{self.name} 执行结果: {result.answer}')
                self.goal_handle = None
                return result.answer
            else:
                self.logger.info(f'{self.name} 执行失败，状态码: {status}')
                return None

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.logger.info(f'{self.name} 目标被拒绝')
            return
        self.goal_handle = goal_handle
        self.logger.info(f'{self.name} 目标已接受')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.action_result = future.result()
        self.event.set()


class SpeechClient(Node):
    def __init__(self):
        super().__init__('speech_client')
        self.get_logger().info('正在启动语音对话节点...')
        self.recognition_client = StringCommandActionClient(
            self, 'speech_recognition/command')
        self.synthesis_client = StringCommandActionClient(
            self, 'speech_synthesis/command')
        self.thread = Thread(target=self.run)
        self.thread.start()

    def run(self):
        self.running = True
        while self.running:
            text = self.recognition_client.send_goal('')
            if text is not None and text != '':
                self.get_logger().info(f'识别输入：{text}')
                # text2 = text + '对吧？'
                text2 = text
                self.get_logger().info(f'合成输出：{text2}')
                self.synthesis_client.send_goal(text2)


def main():
    rclpy.init()
    node = SpeechClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.running = False
        pass

    rclpy.try_shutdown()