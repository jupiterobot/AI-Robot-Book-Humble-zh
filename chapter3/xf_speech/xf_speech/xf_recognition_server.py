# xf_speech/xf_speech_action_server.py

import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from airobot_interfaces.action import StringCommand
from pathlib import Path
import sys

# 抑制 PyAudio / ALSA 的烦人警告
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
try:
    from xf_iat import iat_main
except ImportError as e:
    raise ImportError("无法导入 xf_speech.xf_iat.iat_main，请检查模块路径") from e


class Xf_RecognitionServer(Node):
    def __init__(self):
        super().__init__('xf_recognition_server')
        self.get_logger().info('启动语音识别服务器。')

        self.goal_handle = None
        self.goal_lock = threading.Lock()
        self.execute_lock = threading.Lock()

        # 新增：用于中断录音/识别的事件对象
        self.current_stop_event = None
        self.stop_event_lock = threading.Lock()

        self.action_server = ActionServer(
            self,
            StringCommand,
            'xf_recognition/command',
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            callback_group=ReentrantCallbackGroup(),
        )

    def handle_accepted_callback(self, goal_handle):
        """确保同一时间只有一个活跃目标"""
        with self.goal_lock:
            if self.goal_handle is not None and self.goal_handle.is_active:
                self.get_logger().info('中止之前的语音输入')
                self.goal_handle.abort()
            self.goal_handle = goal_handle
        goal_handle.execute()

    def execute_callback(self, goal_handle):
        with self.execute_lock:
            self.get_logger().info('开始执行语音识别...')
            result = StringCommand.Result()
            result.answer = 'NG'

            RECORD_DURATION = 10  # 秒

            if not goal_handle.is_active or goal_handle.is_cancel_requested:
                self.get_logger().info('任务在开始前已被取消或中止')
                goal_handle.canceled()
                return result

            # 创建新的 stop_event，并保存到实例变量（加锁）
            stop_event = threading.Event()
            with self.stop_event_lock:
                self.current_stop_event = stop_event

            self.get_logger().info(f'开始 {RECORD_DURATION} 秒录音...')

            try:
                # 调用支持中断的 iat_main
                text = iat_main(input_t=RECORD_DURATION, stop_event=stop_event)
            except Exception as e:
                self.get_logger().error(f'识别过程中发生异常: {e}')
                with self.stop_event_lock:
                    self.current_stop_event = None
                return result

            # 清理 stop_event
            with self.stop_event_lock:
                self.current_stop_event = None

            # 检查是否被取消（即使识别完成，也可能在返回前被取消）
            if not goal_handle.is_active:
                self.get_logger().info('任务在识别完成后已被中止')
                return result

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('任务在识别完成后被取消')
                return result

            # 成功
            goal_handle.succeed()
            result.answer = text if text else 'NG'
            self.get_logger().info(f'识别结果: "{text}"')
            return result

    def cancel_callback(self, goal_handle):
        self.get_logger().info('收到取消请求')

        # 触发中断事件
        with self.stop_event_lock:
            if self.current_stop_event is not None:
                self.current_stop_event.set()
                self.get_logger().info('已发送中断信号给录音/识别线程')

        return CancelResponse.ACCEPT


def main():
    rclpy.init()
    node = Xf_RecognitionServer()
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