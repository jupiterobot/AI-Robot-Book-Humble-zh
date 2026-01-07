import random
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer   # 导入动作服务器类
from happy_action_interfaces.action import Happy  # 导入自定义动作定义


class HappyActionServer(Node):
    def __init__(self):
        super().__init__('happy_action_server')  # 使用节点名 'happy_action_server' 初始化
        # 创建动作服务器（动作类型, 动作名称, 回调函数）
        self._action_server = ActionServer(
            self, Happy, 'happy', self.execute_callback)
        self.happy_actions = ({
            1:'对他人做出友善的行为。',
            2:'与他人建立联系。',
            3:'为了健康而运动。',
            4:'进行正念练习。',
            5:'挑战新事物。',
            6:'设定目标，并迈出第一步。',
            7:'培养心理韧性（恢复力）。',
            8:'关注事物积极的一面。',
            9:'接纳人与人之间的差异。',
            10:'大家携手让世界变得更美好。'})

    def execute_callback(self, goal_handle):  # 接收到目标时调用的回调函数
        self.get_logger().info('正在处理目标...')

        feedback_msg = Happy.Feedback()  # 创建反馈消息实例
        feedback_msg.remaining_time = random.randrange(1,10)  # 随机生成剩余时间

        no = goal_handle.request.action_no  # 从目标中获取动作编号
        self.get_logger().info(f'收到目标：[{no}号] {self.happy_actions[no]}')   
        
        while True:  # 目标处理循环
            if  feedback_msg.remaining_time == 0:
                self.get_logger().info('目标处理已完成。')
                break
            else:
                feedback_msg.remaining_time -= 1
                self.get_logger().info(f'反馈：剩余 {feedback_msg.remaining_time} 秒')
                goal_handle.publish_feedback(feedback_msg)  # 发布反馈
                time.sleep(1)
            
        goal_handle.succeed()  # 通知目标成功完成
        result = Happy.Result()  # 创建结果消息实例
        if feedback_msg.remaining_time == 0:  # 设置结果消息内容
            result.happy_result = '变得非常开心了。'
        else:
            result.happy_result = '变得稍微开心了一点。'
        self.get_logger().info(f'返回结果：{result.happy_result}')   # 将结果输出到日志
        return result  # 返回结果

def main():
    rclpy.init()
    happy_action_server = HappyActionServer()
    try:
        rclpy.spin(happy_action_server)
    except KeyboardInterrupt:  # 注意：此处原代码拼写错误 "KeyBoardInterrupt" 已修正为标准写法
        pass
    finally:
        happy_action_server.destroy_node()
        rclpy.shutdown()