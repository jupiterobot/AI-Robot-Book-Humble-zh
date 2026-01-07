import threading
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from airobot_interfaces.action import StringCommand


class SpeechSynthesisClient(Node):
    def __init__(self):
        super().__init__('speech_synthesis_client')
        self.get_logger().info('启动语音合成客户端。')
        self.goal_handle = None
        self.action_client = ActionClient(
            self, StringCommand, 'speech_synthesis/command')

    def say(self, s):
        """发送语音合成请求"""
        self.get_logger().info('等待动作服务器...')
        self.action_client.wait_for_server()
        goal_msg = StringCommand.Goal()
        goal_msg.command = s
        self.get_logger().info('发送目标...')
        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """处理目标是否被接受的回调"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('目标被拒绝。')
            return
        self.goal_handle = goal_handle
        self.get_logger().info('目标已被接受。')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """处理动作执行结果的回调"""
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'结果: {result.answer}')
            self.goal_handle = None
        else:
            self.get_logger().info(f'执行失败，状态码: {status}')

    def cancel_done(self, future):
        """处理取消请求的响应"""
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('取消成功。')
            self.goal_handle = None
        else:
            self.get_logger().info('取消失败。')

    def cancel(self):
        """取消当前正在执行的目标"""
        if self.goal_handle is None:
            self.get_logger().info('没有可取消的任务。')
            return
        self.get_logger().info('正在取消任务...')
        future = self.goal_handle.cancel_goal_async()
        future.add_done_callback(self.cancel_done)


def main():
    # 初始化 ROS 客户端
    rclpy.init()

    # 创建节点实例
    node = SpeechSynthesisClient()

    # 在独立线程中运行 rclpy.spin()
    thread = threading.Thread(target=rclpy.spin, args=(node,))
    threading.excepthook = lambda x: ()  # 忽略线程异常（可选）
    thread.start()

    try:
        while True:
            s = input('> ')
            if s == '':
                # 输入空行时取消当前任务
                node.cancel()
            else:
                # 发送语音合成请求
                node.say(s)
    except KeyboardInterrupt:
        pass

    # 关闭 ROS
    rclpy.try_shutdown()