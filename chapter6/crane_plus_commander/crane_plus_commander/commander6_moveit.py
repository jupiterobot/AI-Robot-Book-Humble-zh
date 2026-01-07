import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from airobot_interfaces.action import StringCommand
import threading
from pymoveit2 import MoveIt2, GripperInterface, MoveIt2State
from math import radians

GRIPPER_MIN = -radians(40.62) + 0.001
GRIPPER_MAX = radians(38.27) - 0.001

def to_gripper_ratio(gripper):
    ratio = (gripper - GRIPPER_MIN) / (GRIPPER_MAX - GRIPPER_MIN)
    return ratio

def from_gripper_ratio(ratio):
    gripper = GRIPPER_MIN + ratio * (GRIPPER_MAX - GRIPPER_MIN)
    return gripper


# 接收来自其他节点的动作请求，并向 CRANE+ V2 的 MoveIt 发送指令的节点
class CommanderMoveit(Node):

    def __init__(self):
        super().__init__('commander_moveit')
        self.joint_names = [
            'crane_plus_joint1',
            'crane_plus_joint2',
            'crane_plus_joint3',
            'crane_plus_joint4']
        callback_group = ReentrantCallbackGroup()
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=self.joint_names,
            base_link_name='crane_plus_base',
            end_effector_name='crane_plus_link_endtip',
            group_name='arm',
            callback_group=callback_group,
        )
        self.moveit2.max_velocity = 1.0
        self.moveit2.max_acceleration = 1.0

        gripper_joint_names = ['crane_plus_joint_hand']
        self.gripper_interface = GripperInterface(
            node=self,
            gripper_joint_names=gripper_joint_names,
            open_gripper_joint_positions=[GRIPPER_MIN],
            closed_gripper_joint_positions=[GRIPPER_MAX],
            gripper_group_name='gripper',
            callback_group=callback_group,
        )
        self.gripper_interface.max_velocity = 1.0
        self.gripper_interface.max_acceleration = 1.0

        # 存储字符串与姿态组合的字典
        self.poses = {}
        self.poses['zeros'] = [0, 0, 0, 0]
        self.poses['ones'] = [1, 1, 1, 1]
        self.poses['home'] = [0.0, -1.16, -2.01, -0.73]
        self.poses['carry'] = [-0.00, -1.37, -2.52, 1.17]

        # 动作服务器
        self.action_server = ActionServer(
            self,
            StringCommand,
            'manipulation/command',
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            callback_group=callback_group,
        )
        self.goal_handle = None               # 保存当前正在处理的目标信息的变量
        self.goal_lock = threading.Lock()     # 防止重复执行的锁变量
        self.execute_lock = threading.Lock()  # 防止重复执行的锁变量

    def handle_accepted_callback(self, goal_handle):
        with self.goal_lock:                  # 确保该代码块不会被重复执行
            if self.goal_handle is not None and self.goal_handle.is_active:
                self.get_logger().info('中止前一个操作')
                self.goal_handle.abort()
                self.cancel_joint_and_gripper()
            self.goal_handle = goal_handle   # 更新目标信息
        goal_handle.execute()                # 执行目标处理

    def execute_callback(self, goal_handle):
        with self.execute_lock:              # 确保该代码块不会被重复执行
            self.get_logger().info(f'command: {goal_handle.request.command}')
            result = StringCommand.Result()
            words = goal_handle.request.command.split()
            if words[0] == 'set_pose':
                self.set_pose(words, result)
            elif words[0] == 'set_gripper':
                self.set_gripper(words, result)
            else:
                result.answer = f'NG {words[0]} not supported'
            self.get_logger().info(f'answer: {result.answer}')
            if goal_handle.is_active:
                if result.answer.startswith('OK'):
                    goal_handle.succeed()
                else:
                    goal_handle.abort()
            return result

    def set_pose(self, words, result):
        if len(words) < 2:
            result.answer = f'NG {words[0]} argument required'
            return
        if not words[1] in self.poses:
            result.answer = f'NG {words[1]} not found'
            return
        self.set_max_velocity(0.5)
        success = self.move_joint(self.poses[words[1]])
        if success:
            result.answer = 'OK'
        else:
            result.answer = f'NG {words[0]} move_joint() failed'

    def set_gripper(self, words, result):
        if len(words) < 2:
            result.answer = f'NG {words[0]} argument required'
            return
        try:
            gripper_ratio = float(words[1])
        except ValueError:
            result.answer = f'NG {words[1]} unsuitable'
            return
        if gripper_ratio < 0.0 or 1.0 < gripper_ratio:
            result.answer = 'NG out of range'
            return
        gripper = from_gripper_ratio(gripper_ratio)
        self.set_max_velocity(0.5)
        success = self.move_gripper(gripper)
        if success:
            result.answer = 'OK'
        else:
            result.answer = f'NG {words[0]} move_gripper() failed'

    def cancel_callback(self, goal_handle):
        self.get_logger().info('收到取消请求')
        self.cancel_joint_and_gripper()
        return CancelResponse.ACCEPT

    def cancel_joint_and_gripper(self):
        if self.moveit2.query_state() == MoveIt2State.EXECUTING:
            self.moveit2.cancel_execution()
        if self.gripper_interface.query_state() == MoveIt2State.EXECUTING:
            self.gripper_interface.cancel_execution()

    def move_joint(self, q):
        joint_positions = [
            float(q[0]), float(q[1]), float(q[2]), float(q[3])]
        self.moveit2.move_to_configuration(joint_positions)
        return self.moveit2.wait_until_executed()

    def move_gripper(self, q):
        position = float(q)
        self.gripper_interface.move_to_position(position)
        return self.gripper_interface.wait_until_executed()

    def set_max_velocity(self, v):
        self.moveit2.max_velocity = float(v)


def main():
    print('开始')

    # ROS客户端的初始化
    rclpy.init()

    # 节点类的实例
    commander = CommanderMoveit()

    # 在另一个线程中执行 rclpy.spin()
    executor = MultiThreadedExecutor()
    thread = threading.Thread(target=rclpy.spin, args=(commander, executor,))
    threading.excepthook = lambda x: ()
    thread.start()

    # 缓慢移动到初始姿态
    commander.set_max_velocity(0.2)
    commander.move_joint(commander.poses['home'])
    commander.move_gripper(GRIPPER_MAX)
    print('等待动作客户端请求')

    # 为避免 Ctrl+C 导致错误，捕获 KeyboardInterrupt
    try:
        input('按 Enter 键退出\n')
    except KeyboardInterrupt:
        thread.join()
    else:
        print('停止动作服务器')
        # 缓慢移动到结束姿态
        commander.set_max_velocity(0.2)
        commander.move_joint(commander.poses['zeros'])
        commander.move_gripper(GRIPPER_MIN)

    rclpy.try_shutdown()
    print('结束')