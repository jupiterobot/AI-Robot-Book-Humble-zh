import time, random
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from airobot_interfaces.action import StringCommand  # 导入自定义动作定义


class BringmeActionServer(Node):
    def __init__(self):
        super().__init__('bringme_action_server')
        self._action_server = ActionServer(
            self, StringCommand, 'command', 
            execute_callback=self.execute_callback,
        )
        self.food = ['apple', 'banana', 'candy']

    def execute_callback(self, goal_handle):
        feedback = StringCommand.Feedback()
        count = random.randint(5, 10)

        while count > 0:
            self.get_logger().info(f'正在发送反馈：剩余 {count} 秒')     
            feedback.process = f'{count}'
            goal_handle.publish_feedback(feedback)  
            count -= 1  
            time.sleep(1)

        item = goal_handle.request.command
        result = StringCommand.Result()
        if item in self.food:
            result.answer = f'好的，这是 {item}。'
        else:
            result.answer = f'抱歉，找不到 {item}。'
        goal_handle.succeed()
        self.get_logger().info(f'目标执行结果：{result.answer}')
        return result


def main():
    rclpy.init()
    bringme_action_server = BringmeActionServer()
    print('服务已启动')
    try:
        rclpy.spin(bringme_action_server)
    except KeyboardInterrupt:
        pass
    rclpy.try_shutdown()
    print('服务已结束')