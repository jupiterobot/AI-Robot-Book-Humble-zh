import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from airobot_interfaces.action import StringCommand

import random
from time import sleep


class VoiceActionServer(Node):
    def __init__(self):
        super().__init__('voice_action_server')
        self._voice_action_server = ActionServer(   # 创建 ActionServer
            self,                                   # 指定 ROS 节点
            StringCommand,                          # 指定 Action 类型
            'ps_voice/command',                     # 指定 Action 名称
            self.execute_callback)                  # 将接收到的 goal 交由 callback 函数处理

    def execute_callback(self, goal_handle):
        self.get_logger().info(f'将在 `{goal_handle.request.command}` 秒内开始语音识别')

        result_msg = StringCommand.Result()     # 定义针对输入 goal 的结果消息类型
        result_msg.answer = ''                  # 初始化 Action 的结果

        feedback_msg = StringCommand.Feedback() # 定义针对输入 goal 的中间反馈消息类型
        feedback_msg.process = ''               # 初始化 Action 的中间反馈

        wait_time = int(goal_handle.request.command) # 接收到的命令是 String 类型，因此转换为 int 类型

        # --- 此部分用于代替实际的语音处理，在实际系统中是未知的 ---
        unk_text = 'bring me a cup from the kitchen'    # 将语音识别得到的语音数据转换为文本
        unk_text_split = unk_text.split()               # 将每个单词存入 List 类型
        unk_text_list = unk_text_split + [''] * (wait_time - len(unk_text_split)) # 根据 wait_time 长度，在 List 末尾补充空元素
        temp_out_list = [] # 为中间结果定义临时 List
        temp_out_text = '' # 为中间结果定义临时文本变量
        # ------------------------------------------------------------------------

        for i in range(wait_time):
            # 执行语音识别处理

            sleep(1) # 为了可视化处理过程，暂停 1 秒
            prob = random.random() # 生成一个 [0,1] 范围内的随机值

            if 0.95 > prob:
                # [成功] 当 prob 值小于 0.95 时执行（概率 95%）
                self.get_logger().info('正在进行语音识别')
                temp_out_list.append(unk_text_list[i]+ ' ' if unk_text_list[i]!='' else '') # 如果 unk_text_list 的元素非空，则追加到临时 List
                temp_out_text = (''.join(word for word in temp_out_list if word)) # 将 List 转换为 String 类型
                
                feedback_msg.process = temp_out_text # 将 `temp_out_text` 作为中间结果
                goal_handle.publish_feedback(feedback_msg) # 将中间结果作为反馈发布

            else:
                # [失败] 当 prob 值大于等于 0.95 时执行（概率 5%）
                self.get_logger().info('语音识别失败了')

                feedback_msg.process = '' # 由于中途发生失败，将反馈置为空
                goal_handle.publish_feedback(feedback_msg) # 发布中间反馈
                goal_handle.abort() # 因中途失败，强制终止 Action 处理

                result_msg.answer = 'failed' # 由于失败，将最终结果设为 `failed`
                return result_msg
        
        self.get_logger().info('语音识别成功完成')

        goal_handle.succeed() # 报告 goal 已成功完成

        result_msg.answer = feedback_msg.process # 将中间反馈的最后一个值赋给 result_msg
        return result_msg


def main(args=None):
    rclpy.init(args=args)

    voice_action_server = VoiceActionServer() # 声明 VoiceActionServer() 类

    rclpy.spin(voice_action_server) # 启动 ActionServer 的回调函数

    rclpy.shutdown()

if __name__ == '__main__':
    main()