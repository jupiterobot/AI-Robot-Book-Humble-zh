import time
import random
from threading import Lock
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from airobot_interfaces.action import StringCommand  # 导入自定义动作定义


class BringmeActionServer(Node):
    def __init__(self):
        super().__init__('bringme_action_server')
        self.goal_handle = None    # 存储当前正在处理的目标的信息变量
        self.goal_lock = Lock()    # 防止并发执行的锁变量
        self.execute_lock = Lock() # 防止并发执行的锁变量
        self._action_server = ActionServer(
            self, StringCommand, 'command',
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            callback_group=ReentrantCallbackGroup(),
        )
        self.food = ['apple', 'banana', 'candy']  # 定义可识别的食物列表

    def handle_accepted_callback(self, goal_handle):
        with self.goal_lock:  # 确保此块不会并发执行
            if self.goal_handle is not None and self.goal_handle.is_active:
                self.get_logger().info('中断之前的任务')
                self.goal_handle.abort()
            self.goal_handle = goal_handle  # 更新目标信息
        goal_handle.execute()  # 执行目标处理

    def execute_callback(self, goal_handle):
        with self.execute_lock:  # 确保此块不会并发执行
            feedback = StringCommand.Feedback()
            result = StringCommand.Result()
            count = random.randint(5, 10)  # 随机生成一个计数值

            while count > 0:
                if not goal_handle.is_active:
                    self.get_logger().info('检测到中断请求')
                    return result

                if goal_handle.is_cancel_requested:
                    self.get_logger().info('接收到取消请求')
                    goal_handle.canceled()
                    return result

                self.get_logger().info(f'发送反馈：剩余{count}[秒]')
                feedback.process = f'{count}'
                goal_handle.publish_feedback(feedback)
                count -= 1
                time.sleep(1)

            item = goal_handle.request.command
            if item in self.food:
                result.answer = f'是的，找到了{item}'
            else:
                result.answer = f'未能找到{item}'
            goal_handle.succeed()
            self.get_logger().info(f'目标结果: {result.answer}')
            return result

    def cancel_callback(self, goal_handle):
        self.get_logger().info('接收到取消请求')
        return CancelResponse.ACCEPT


def main():
    rclpy.init()
    bringme_action_server = BringmeActionServer()
    print('服务器启动')
    try:
        rclpy.spin(bringme_action_server, executor=MultiThreadedExecutor())
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
    print('服务器关闭')