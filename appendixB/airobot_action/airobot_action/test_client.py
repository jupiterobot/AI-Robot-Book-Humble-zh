import os
import sys
import readline  # 为 input() 添加命令历史功能所必需
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.utilities import remove_ros_args
from action_msgs.msg import GoalStatus
from airobot_interfaces.action import StringCommand


class TestClient(Node):
    def __init__(self, action_name):
        super().__init__('test_client')
        self.get_logger().info(f'正在启动 {action_name} 的客户端')
        self.goal_handle = None  # 存储当前正在处理的目标信息
        self.action_client = ActionClient(
            self, StringCommand, action_name)
        while not self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('动作服务器未就绪，等待中...')

    def send_goal(self, command):
        self.get_logger().info(f'发送目标: {command}')
        goal_msg = StringCommand.Goal()
        goal_msg.command = command
        self.send_goal_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'收到反馈: \'{feedback_msg.feedback.process}\'')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('目标被拒绝')
            return
        self.goal_handle = goal_handle  # 更新当前目标信息
        self.get_logger().info('目标已接受')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'执行结果: {result.answer}')
        else:
            self.get_logger().info(f'执行失败，状态码: {status}')
        self.goal_handle = None  # 重置目标信息

    def cancel(self):
        if self.goal_handle is None:
            self.get_logger().info('无可取消的目标')
            return
        self.get_logger().info('正在请求取消...')
        future = self.goal_handle.cancel_goal_async()
        future.add_done_callback(self.cancel_done)

    def cancel_done(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('取消成功')
            self.goal_handle = None  # 重置目标信息
        else:
            self.get_logger().info('取消失败')


def main():
    action_name = 'command'                # 动作名称默认值
    args = remove_ros_args(args=sys.argv)  # 从命令行参数中移除 ROS 特有参数
    if len(args) >= 2:                     # 使用第一个非 ROS 参数作为动作名
        action_name = args[1]
    history_path = '.history_' + action_name.replace('/', '_')
    if os.path.isfile(history_path):
        readline.read_history_file(history_path)

    print('''使用说明：
  输入任意字符串后按 Enter → 作为 StringCommand 的 command 目标发送
  直接按 Enter（空输入） → 取消当前正在执行的目标
  输入 "exit" 后按 Enter → 退出程序
  支持在服务器执行过程中发送新目标（旧目标将被自动中断）
  支持命令历史记录与行编辑功能（上下箭头可切换历史命令）
''')

    rclpy.init()
    node = TestClient(action_name)
    thread = threading.Thread(target=rclpy.spin, args=(node,))
    threading.excepthook = lambda x: ()  # 避免线程异常打印堆栈
    thread.start()

    try:
        while True:
            command = input('command: ')
            if command == '':        # 空输入 → 取消
                node.cancel()
            elif command == 'exit':  # 输入 exit → 退出
                break
            else:                    # 其他输入 → 发送为目标
                node.send_goal(command)
    except KeyboardInterrupt:
        pass

    rclpy.try_shutdown()
    readline.write_history_file(history_path)  # 保存命令历史