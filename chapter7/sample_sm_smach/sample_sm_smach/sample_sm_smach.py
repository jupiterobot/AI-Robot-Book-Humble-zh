# 文件名：sample_sm.py

import rclpy  # [*] 导入用于在 Python 中使用 ROS 2 的模块。
from rclpy.node import Node
import smach  # [*] 导入用于创建状态机的 Smach 模块。


# 定义“搜索”状态。
class Search(smach.State):
    def __init__(self, _node):
        smach.State.__init__(self, outcomes=['succeeded', 'finished'])  # [*] 预先定义该状态可能的输出结果。
        self.counter = 0  # [*] 记录进入此状态的次数。
        self.logger = _node.get_logger()  # [*] 获取日志记录器。

    def execute(self, userdata):
        self.logger.info('正在搜索')  # [*] 记录当前处于搜索状态。
        if self.counter < 3:
            self.logger.info('找到甜点了！')  # [*] 若进入次数少于 3 次
            self.counter += 1
            return 'succeeded'  # [*] 返回 'succeeded' 结果。
        else:
            self.logger.info('已经吃饱了……')  # [*] 若已进入 3 次
            return 'finished'  # [*] 返回 'finished' 结果。


# 定义“进食”状态。
class Eat(smach.State):
    def __init__(self, _node):
        smach.State.__init__(self, outcomes=['done'])  # [*] 预先定义该状态的输出结果。
        self.logger = _node.get_logger()  # [*] 获取日志记录器。

    def execute(self, userdata):
        self.logger.info('正在吃！')  # [*] 记录当前处于进食状态。
        return 'done'  # [*] 返回 'done' 结果。


# 定义执行状态机的 ROS 2 节点。
class StateMachine(Node):
    def __init__(self):
        super().__init__('state_machine')  # [*] 将节点注册为名为 'state_machine' 的节点。

    def execute(self):
        # 创建 Smach 状态机，定义最终可能的结果为 'end'
        sm = smach.StateMachine(outcomes=['end'])
        # 打开状态机容器以定义状态之间的连接
        with sm:  # [*] 在此上下文中定义状态间的转换关系。
            # 添加 'SEARCH' 状态
            smach.StateMachine.add(
                'SEARCH', Search(self),
                transitions={'succeeded': 'EAT', 'finished': 'end'})
            # 添加 'EAT' 状态
            smach.StateMachine.add(
                'EAT', Eat(self),
                transitions={'done': 'SEARCH'})

        # 执行状态机
        outcome = sm.execute()
        self.get_logger().info(f'最终结果: {outcome}')


def main():
    rclpy.init()  # [*] 初始化 rclpy，启用 ROS 2 通信。
    node = StateMachine()  # [*] 初始化状态机节点。
    node.execute()  # [*] 执行状态机。