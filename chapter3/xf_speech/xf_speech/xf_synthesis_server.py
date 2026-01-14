# xf_speech/xf_speech_action_server.py （更新版）

import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from airobot_interfaces.action import StringCommand
from pathlib import Path
import sys

# 抑制 ALSA 警告
from ctypes import CFUNCTYPE, c_char_p, c_int, cdll
ERROR_HANDLER_FUNC = CFUNCTYPE(None, c_char_p, c_int, c_char_p, c_int, c_char_p)
def py_error_handler(filename, line, function, err, fmt):
    pass
c_error_handler = ERROR_HANDLER_FUNC(py_error_handler)
try:
    asound = cdll.LoadLibrary('libasound.so')
    asound.snd_lib_error_set_handler(c_error_handler)
except OSError:
    pass

pkg_path = str(Path(__file__).resolve().parents[1])
sys.path.insert(0, pkg_path + "/xf_speech")
from xf_tts import tts_main


class Xf_SynthesisServer(Node):
    def __init__(self):
        super().__init__('xf_synthesis_server')
        self.get_logger().info('启动语音合成服务器。')
        self.goal_handle = None
        self.goal_lock = threading.Lock()
        self.execute_lock = threading.Lock()
        self.current_stop_event = None

        self.action_server = ActionServer(
            self,
            StringCommand,
            'xf_synthesis/command',
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            callback_group=ReentrantCallbackGroup(),
        )

    def handle_accepted_callback(self, goal_handle):
        with self.goal_lock:
            if self.goal_handle is not None and self.goal_handle.is_active:
                self.get_logger().info('中止之前的语音播报')
                self.goal_handle.abort()
                if self.current_stop_event:
                    self.current_stop_event.set()
            self.goal_handle = goal_handle
        goal_handle.execute()

    def execute_callback(self, goal_handle):
        with self.execute_lock:
            result = StringCommand.Result()
            result.answer = 'NG'

            text = goal_handle.request.command.strip()
            if not text:
                self.get_logger().warn('收到空文本')
                goal_handle.succeed()
                result.answer = 'OK'
                return result

            self.get_logger().info(f'开始合成: "{text}"')

            # 创建新的停止事件
            stop_event = threading.Event()
            self.current_stop_event = stop_event

            try:
                # 调用可中断的 TTS
                tts_main(text, stop_event=stop_event)
            except Exception as e:
                self.get_logger().error(f'TTS 异常: {e}')
                if not stop_event.is_set():
                    # 如果不是因为取消而失败，才报错
                    return result

            # 检查最终状态
            if not goal_handle.is_active:
                self.get_logger().info('任务被中止')
                return result

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('任务被取消')
                return result

            goal_handle.succeed()
            result.answer = 'OK'
            self.get_logger().info('语音合成完成')
            return result

    def cancel_callback(self, goal_handle):
        self.get_logger().info('收到取消请求')
        if self.current_stop_event:
            self.current_stop_event.set()
        return CancelResponse.ACCEPT


def main():
    rclpy.init()
    node = Xf_SynthesisServer()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()