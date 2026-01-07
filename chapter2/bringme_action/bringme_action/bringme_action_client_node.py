import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from airobot_interfaces.action import StringCommand

class BringmeActionClient(Node):
    def __init__(self):  # 构造函数
        super().__init__('bringme_action_client')
        # 初始化动作客户端
        self._action_client = ActionClient(self, StringCommand, 'command')

    def send_goal(self, order):  # 发送目标
        goal_msg = StringCommand.Goal()  # 创建目标消息
        goal_msg.command = order        
        self._action_client.wait_for_server()  # 等待服务器准备就绪
        # 异步发送目标，并设置反馈回调
        return self._action_client.send_goal_async(  
            goal_msg, feedback_callback=self.feedback_callback
        )

    def feedback_callback(self, feedback_msg):  # 接收反馈并显示进度
        self.get_logger().info(f'正在接收反馈：剩余 {feedback_msg.feedback.process} 秒')


def main():
    rclpy.init()
    bringme_action_client = BringmeActionClient()
    order = input('要拿什么过来？')
    
    # 发送目标并获取 Future 对象
    future = bringme_action_client.send_goal(order)  
    # 等待目标发送完成
    rclpy.spin_until_future_complete(bringme_action_client, future)      
    goal_handle = future.result()  # 获取目标句柄

    if not goal_handle.accepted:
        bringme_action_client.get_logger().info('目标已被拒绝')
    else:
        bringme_action_client.get_logger().info('目标已接受')        
        result_future = goal_handle.get_result_async()  # 异步获取结果
        # 等待结果返回
        rclpy.spin_until_future_complete(bringme_action_client, result_future)        
        result = result_future.result().result  # 获取结果        
        bringme_action_client.get_logger().info(f'目标执行结果：{result.answer}')
    
    bringme_action_client.destroy_node()
    rclpy.shutdown()