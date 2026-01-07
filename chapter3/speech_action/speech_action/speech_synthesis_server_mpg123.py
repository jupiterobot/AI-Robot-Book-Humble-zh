import threading
import rclpy
import subprocess
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from airobot_interfaces.action import StringCommand
from gtts import gTTS
from io import BytesIO
from mpg123 import Mpg123, Out123



class SpeechSynthesisServer(Node):
    def __init__(self):
        super().__init__('speech_synthesis_server')
        self.get_logger().info('启动语音合成服务器。')
        # self.lang = 'zh-CN'
        self.lang = 'en'
        self.out = Out123()
        self.goal_handle = None
        self.goal_lock = threading.Lock()
        self.execute_lock = threading.Lock()
        self.action_server = ActionServer(
            self,
            StringCommand,
            'speech_synthesis/command',
            self.execute_callback,
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            callback_group=ReentrantCallbackGroup(),
        )

    def handle_accepted_callback(self, goal_handle):
        with self.goal_lock:
            if self.goal_handle is not None and self.goal_handle.is_active:
                self.get_logger().info('中止之前的语音播报')
                self.goal_handle.abort()
            self.goal_handle = goal_handle
        goal_handle.execute()

    def execute_callback(self, goal_handle):
        with self.execute_lock:
            self.get_logger().info('执行...')
            result = StringCommand.Result()
            result.answer = 'NG'
            if goal_handle.request.command != '':
                text = goal_handle.request.command
                self.get_logger().info(f'播报： {text}')


                gTTS(text, lang='en').save('voice.mp3')
                subprocess.run(['mpg123 voice.mp3'], shell=True)
                
                if not goal_handle.is_active:
                    self.get_logger().info('中止')
                    return result

                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info('取消')
                    return result

                goal_handle.succeed()
                result.answer = 'OK'
            return result

    def cancel_callback(self, goal_handle):
        self.get_logger().info('收到取消请求')
        return CancelResponse.ACCEPT


def main():
    rclpy.init()
    node = SpeechSynthesisServer()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass

    rclpy.try_shutdown()