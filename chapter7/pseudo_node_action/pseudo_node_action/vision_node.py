import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from airobot_interfaces.action import StringCommand

import random
from time import sleep


class VisionActionServer(Node):
    def __init__(self):
        super().__init__('vision_action_server')
        self._vision_action_server = ActionServer(  # 创建 ActionServer
            self,                                   # 指定 ROS 节点
            StringCommand,                          # 指定 Action 类型
            'ps_vision/command',                    # 指定 Action 名称
            self.execute_callback)                  # 将接收到的 goal 交由 callback 函数处理

    def execute_callback(self, goal_handle):
        self.get_logger().info(f'开始检测目标物体 `{goal_handle.request.command}`')

        result_msg = StringCommand.Result()     # 定义针对输入 goal 的结果消息类型
        result_msg.answer = ''                  # 初始化 Action 的结果

        feedback_msg = StringCommand.Feedback() # 定义针对输入 goal 的中间反馈消息类型
        feedback_msg.process = ''               # 初始化 Action 的中间反馈

        wait_time = 10 # 定义模拟处理所需的秒数

        for i in range(wait_time):
            # 执行物体识别处理

            sleep(1) # 为了可视化处理过程，暂停 1 秒
            prob = random.random() # 生成一个 [0,1] 范围内的随机值

            if 0.95 > prob:
                # [成功] 当 prob 值小于 0.95 时执行（概率 95%）
                self.get_logger().info(f'正在寻找目标物体 `{goal_handle.request.command}`')

                feedback_msg.process = 'finding' if i < wait_time-1 else 'found' # 中间反馈设为 `finding`，最终设为 `found`
                goal_handle.publish_feedback(feedback_msg) # 将中间结果作为反馈发布

            else:
                self.get_logger().info(f'检测目标物体 `{goal_handle.request.command}` 失败了')

                feedback_msg.process = '' # 由于中途发生失败，将反馈置为空
                goal_handle.publish_feedback(feedback_msg) # 发布中间反馈
                goal_handle.abort() # 因中途失败，强制终止 Action 处理

                result_msg.answer = 'failed' # 由于失败，将最终结果设为 `failed`
                return result_msg

        self.get_logger().info(f'成功检测到目标物体 `{goal_handle.request.command}`')

        goal_handle.succeed() # 报告 goal 已成功完成

        result_msg.answer = feedback_msg.process # 将中间反馈的最后一个值赋给 result_msg
        return result_msg


def main(args=None):
    rclpy.init(args=args)

    vision_action_server = VisionActionServer() # 声明 VisionActionServer() 类

    rclpy.spin(vision_action_server) # 启动 ActionServer 的回调函数

    rclpy.shutdown()

if __name__ == '__main__':
    main()