import rclpy #[*] 导入用于在 Python 中使用 ROS2 的模块。
from rclpy.node import Node
import smach #[*] 用于创建状态机的模块。

from airobot_interfaces.srv import StringCommand #[*] 导入用于服务通信的数据类型。


# 定义执行 Bring me 任务的状态机节点
class Bringme_state(Node):
    def __init__(self):
        super().__init__('bringme_state') #[*] 将节点名注册为 bringme_state。

    def execute(self):
        # 创建 Smach 状态机
        sm = smach.StateMachine(outcomes=['succeeded'])

        # 向容器中添加状态
        with sm: #[*] 定义各状态之间的连接关系。
            smach.StateMachine.add(
                'VOICE',
                Voice(self),
                {'succeeded': 'NAVIGATION', 'failed': 'VOICE'})

            smach.StateMachine.add(
                'NAVIGATION',
                Navigation(self),
                {'succeeded': 'VISION', 'failed': 'NAVIGATION'})

            smach.StateMachine.add(
                'VISION',
                Vision(self),
                {'succeeded': 'MANIPULATION', 'failed': 'VISION'})

            smach.StateMachine.add(
                'MANIPULATION',
                Manipulation(self),
                {'failed': 'VISION', 'exit': 'succeeded'})

        # 执行 Smach 计划
        sm.execute()


def main():
    rclpy.init() #[*] 初始化 rclpy 以启用 ROS 通信。
    node = Bringme_state() #[*] 初始化状态机节点。
    node.execute() #[*] 执行状态机。


# 音声识别相关状态
class Voice(smach.State):
    def __init__(self, node):
        smach.State.__init__( #[*] 预先定义音声识别状态的结果，以及向其他状态传递值时使用的名称。
            self,
            output_keys=['target_object', 'target_location'],
            outcomes=['succeeded', 'failed'])

        # 创建 Node
        self.node = node
        self.logger = self.node.get_logger() #[*] 定义日志记录器。
        
        # 创建服务客户端
        self.cli = self.node.create_client(StringCommand, 'voice/command')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.logger.info('正在等待连接到服务...')
        self.req = StringCommand.Request()

        self.result = None

    def execute(self, userdata):
        self.logger.info('开始音声识别状态') #[*] 当执行音声识别状态时，记录日志。

        self.req.command = 'start' #[*] 发送请求以启动音声识别服务。
        result = self.send_request()

        if result: #[*] 如果从语音识别获得了识别结果
            userdata.target_object = self.target_object #[*] 将识别出的物体和位置名称存入用于向其他状态传递值的变量中。
            userdata.target_location = self.target_location

            return 'succeeded' #[*] 返回成功。
        else:
            return 'failed' #[*] 返回失败。

    def send_request(self):
        self.future = self.cli.call_async(self.req) #[*] 创建与服务器通信的客户端。

        # 执行服务调用的处理
        while rclpy.ok():
            rclpy.spin_once(self.node)
            if self.future.done():
                response = self.future.result()
                break

        if response.answer != '': #[*] 如果响应的 answer 字段包含非空字符串
            self.target_object = 'cup'  #[*] 将字符串赋值给类内变量。find_object_name(response.answer)
            self.target_location = 'kitchen'  #[*] 将字符串赋值给类内变量。find_location_name(response.answer)

            return True
        else:
            return False


# 导航状态
class Navigation(smach.State):
    def __init__(self, node):
        smach.State.__init__( #[*] 预先定义导航状态的结果，以及向其他状态传递值时使用的名称。
            self,
            input_keys=['target_location'],
            outcomes=['succeeded', 'failed'])

        # 创建 Node
        self.node = node
        self.logger = self.node.get_logger() #[*] 定义日志记录器。
        
        # 创建服务客户端
        self.cli = self.node.create_client(StringCommand, 'navigation/command')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.logger.info('正在等待连接到服务...')
        self.req = StringCommand.Request()

        self.result = None

    def execute(self, userdata):
        self.logger.info('开始导航状态') #[*] 当执行导航状态时，记录日志。

        self.req.command = userdata.target_location #[*] 发送请求以执行导航服务。
        result = self.send_request()

        if result: #[*] 如果成功到达目标位置
            return 'succeeded' #[*] 返回成功。
        else:
            return 'failed' #[*] 返回失败。

    def send_request(self):
        self.future = self.cli.call_async(self.req) #[*] 创建与服务器通信的客户端。

        # 执行服务调用的处理
        while rclpy.ok():
            rclpy.spin_once(self.node)
            if self.future.done():
                response = self.future.result()
                break

        if response.answer == 'reached': #[*] 如果响应的 answer 字段为 'reached'
            return True
        else:
            return False


# 视觉状态
class Vision(smach.State):
    def __init__(self, node):
        smach.State.__init__( #[*] 预先定义视觉状态的结果，以及向其他状态传递值时使用的名称。
            self,
            input_keys=['target_object'],
            output_keys=['target_object_pos'],
            outcomes=['succeeded', 'failed'])

        # 创建 Node
        self.node = node
        self.logger = self.node.get_logger() #[*] 定义日志记录器。

        # 创建服务客户端
        self.cli = self.node.create_client(StringCommand, 'vision/command')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.logger.info('正在等待连接到服务...')
        self.req = StringCommand.Request()

        self.result = None

    def execute(self, userdata):
        self.logger.info('开始视觉状态') #[*] 当执行视觉状态时，记录日志。

        self.req.command = userdata.target_object #[*] 发送请求以执行物体识别服务。
        result = self.send_request()
        userdata.target_object_pos = [0.12, -0.03, 0.4]   #[*] 假设物体识别成功，将物体位置赋值（单位：[m]）。

        if result: #[*] 如果成功识别到物体
            return 'succeeded'
        else:
            return 'failed'

    def send_request(self):
        self.future = self.cli.call_async(self.req) #[*] 创建与服务器通信的客户端。

        # 执行服务调用的处理
        while rclpy.ok():
            rclpy.spin_once(self.node)
            if self.future.done():
                response = self.future.result()
                break

        if response.answer == 'detected': #[*] 如果响应的 answer 字段为 'detected'
            return True #[*] 返回成功。
        else:
            return False #[*] 返回失败。


# 操作（机械臂控制）状态
class Manipulation(smach.State):
    def __init__(self, node):
        smach.State.__init__( #[*] 预先定义操作状态的结果，以及向其他状态传递值时使用的名称。
            self,
            input_keys=['target_object_pos'],
            outcomes=['exit', 'failed'])

        # 创建 Node
        self.node = node
        self.logger = self.node.get_logger() #[*] 定义日志记录器。

        # 创建服务客户端
        self.cli = self.node.create_client(
            StringCommand, 'manipulation/command')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.logger.info('正在等待连接到服务...')
        self.req = StringCommand.Request()

        self.result = None

    def execute(self, userdata):
        self.logger.info('开始操作状态') #[*] 当执行操作状态时，记录日志。

        target_object_pos = userdata.target_object_pos #[*] 赋值机械臂要伸向的目标坐标。
        print(f'{target_object_pos}')

        self.req.command = 'start' #[*] 发送请求以执行操作服务。
        result = self.send_request()

        if result: #[*] 如果机械臂完成到达动作
            return 'exit'
        else:
            return 'failed'

    def send_request(self):
        self.future = self.cli.call_async(self.req) #[*] 创建与服务器通信的客户端。

        # 执行服务调用的处理
        while rclpy.ok():
            rclpy.spin_once(self.node)
            if self.future.done():
                response = self.future.result()
                break

        if response.answer == 'reached': #[*] 如果响应的 answer 字段为 'reached'
            return True #[*] 返回成功。
        else:
            return False #[*] 返回失败。