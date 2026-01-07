import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from happy_action_interfaces.action import Happy


class HappyActionClient(Node):
    def __init__(self):
        super().__init__('happy_action_client')
        # 创建动作客户端（节点对象, 动作类型, 动作名称）
        self._action_client = ActionClient(self, Happy, 'happy')

    def send_goal(self, action_no):
        goal_msg = Happy.Goal()  # 创建目标消息类型的实例
        goal_msg.action_no = action_no  # 为目标消息的 action_no 赋值
        self._action_client.wait_for_server()  # 等待动作服务器可用
        # 异步发送目标到动作服务器，并设置接收反馈的回调函数
        self._send_goal_future = self._action_client.send_goal_async(goal_msg,
                    feedback_callback=self.feedback_callback)
        # 设置用于处理目标发送结果的回调函数
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()  # 获取目标句柄
        if not goal_handle.accepted:  # 如果目标未被接受
            self.get_logger().info('目标被拒绝。')
            return
        self.get_logger().info('目标已被接受。')
        # 异步获取目标执行结果，并在结果返回时调用回调函数
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result  # 获取目标的执行结果
        self.get_logger().info(f'结果：{result.happy_result}')
         
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback  # 获取反馈消息
        self.get_logger().info(f'收到反馈：剩余 {feedback.remaining_time} 秒')


def main(args=None):
    rclpy.init(args=args)  # 初始化 ROS2 通信
    action_client = HappyActionClient()  # 创建节点
    action_client.send_goal(1)  # 发送动作编号 1 的目标
    action_client.send_goal(5)  # 发送动作编号 5 的目标
    action_client.send_goal(7)  # 发送动作编号 7 的目标
    rclpy.spin(action_client)  # 持续运行节点，反复调用回调函数，直到节点被销毁
    action_client.destroy_node()  # 销毁节点
    rclpy.shutdown()  # 关闭 ROS2 通信