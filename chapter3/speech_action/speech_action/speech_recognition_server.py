import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from airobot_interfaces.action import StringCommand
from ctypes import CFUNCTYPE, c_char_p, c_int, cdll
from speech_recognition import (
    Recognizer, Microphone, UnknownValueError, RequestError, WaitTimeoutError)


# 抑制 PyAudio 的警告信息
# https://stackoverflow.com/questions/7088672/pyaudio-working-but-spits-out-error-messages-each-time
ERROR_HANDLER_FUNC = CFUNCTYPE(None, c_char_p, c_int, c_char_p, c_int, c_char_p)
def py_error_handler(filename, line, function, err, fmt):
    pass
c_error_handler = ERROR_HANDLER_FUNC(py_error_handler)
asound = cdll.LoadLibrary('libasound.so')
asound.snd_lib_error_set_handler(c_error_handler)


class SpeechRecognitionServer(Node):
    def __init__(self):
        super().__init__('speech_recognition_server')
        self.get_logger().info('启动语音识别服务器。')
        # self.lang = 'zh'  # 中文
        self.lang = 'en'     # 当前设为英文
        self.recognizer = Recognizer()
        self.goal_handle = None
        self.goal_lock = threading.Lock()
        self.execute_lock = threading.Lock()
        self.action_server = ActionServer(
            self,
            StringCommand,
            'speech_recognition/command',
            self.execute_callback,
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            callback_group=ReentrantCallbackGroup(),
        )

    def handle_accepted_callback(self, goal_handle):
        with self.goal_lock:
            if self.goal_handle is not None and self.goal_handle.is_active:
                self.get_logger().info('中止之前的语音输入')
                self.goal_handle.abort()
            self.goal_handle = goal_handle
        goal_handle.execute()

    def execute_callback(self, goal_handle):
        with self.execute_lock:
            self.get_logger().info('开始执行...')
            result = StringCommand.Result()
            result.answer = 'NG'
            with Microphone() as source:
                self.get_logger().info('正在监听语音输入')
                self.recognizer.adjust_for_ambient_noise(source)
                try:
                    audio_data = self.recognizer.listen(
                        source, timeout=10.0, phrase_time_limit=10.0)
                except WaitTimeoutError:
                    self.get_logger().info('超时：未检测到语音')
                    return result

            if not goal_handle.is_active:
                self.get_logger().info('任务已被中止')
                return result

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('已取消')
                return result

            text = ''
            try:
                self.get_logger().info('正在进行语音识别')
                # [*] 将音频数据发送给 Whisper 模型，并获取识别结果
                text = self.recognizer.recognize_whisper(audio_data, model="medium", language=self.lang)
                # text = self.recognizer.recognize_google(audio_data, language=self.lang)
            except RequestError:
                self.get_logger().info('API 请求失败（例如网络或密钥问题）')
                return result

            except UnknownValueError:
                self.get_logger().info('无法识别语音内容')

            if not goal_handle.is_active:
                self.get_logger().info('任务已被中止')
                return result

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('已取消')
                return result

            goal_handle.succeed()
            result.answer = text
            self.get_logger().info(f'识别结果：{text}')
            return result

    def cancel_callback(self, goal_handle):
        self.get_logger().info('收到取消请求')
        return CancelResponse.ACCEPT


def main():
    rclpy.init()
    node = SpeechRecognitionServer()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass

    rclpy.try_shutdown()