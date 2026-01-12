import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
import time
import threading
from crane_plus_commander.kbhit import KBHit
from crane_plus_commander.kinematics import gripper_in_range, joint_in_range


# 向 CRANE+ V2 发布话题指令的节点，
# 并新增向 CRANE+ V2 动作服务器发送请求的功能，
# 同时增加姿态注册功能。
class Commander(Node):

    def __init__(self):
        super().__init__('commander')
        self.joint_names = [
            'crane_plus_joint1',
            'crane_plus_joint2',
            'crane_plus_joint3',
            'crane_plus_joint4']
        self.publisher_joint = self.create_publisher(
            JointTrajectory,
            'crane_plus_arm_controller/joint_trajectory', 10)
        self.publisher_gripper = self.create_publisher(
            JointTrajectory,
            'crane_plus_gripper_controller/joint_trajectory', 10)
        self.action_client_joint = ActionClient(
            self, FollowJointTrajectory,
            'crane_plus_arm_controller/follow_joint_trajectory')

    def publish_joint(self, q, time):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        msg.points = [JointTrajectoryPoint()]
        msg.points[0].positions = [
            float(q[0]), float(q[1]), float(q[2]), float(q[3])]
        msg.points[0].time_from_start = Duration(
            seconds=int(time), nanoseconds=(time-int(time))*1e9).to_msg()
        self.publisher_joint.publish(msg)

    def publish_gripper(self, gripper, time):
        msg = JointTrajectory()
        msg.joint_names = ['crane_plus_joint_hand']
        msg.points = [JointTrajectoryPoint()]
        msg.points[0].positions = [float(gripper)]
        msg.points[0].time_from_start = Duration(
            seconds=int(time), nanoseconds=(time-int(time))*1e9).to_msg()
        self.publisher_gripper.publish(msg)

    def send_goal_joint(self, q, time):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = JointTrajectory()
        goal_msg.trajectory.joint_names = self.joint_names
        goal_msg.trajectory.points = [JointTrajectoryPoint()]
        goal_msg.trajectory.points[0].positions = [
            float(q[0]), float(q[1]), float(q[2]), float(q[3])]
        goal_msg.trajectory.points[0].time_from_start = Duration(
            seconds=int(time), nanoseconds=(time-int(time))*1e9).to_msg()
        self.action_client_joint.wait_for_server()
        return self.action_client_joint.send_goal(goal_msg)


# 将数值列表格式化为统一小数位数的字符串
def list_to_str(x):
    return '[' + ', '.join([f'{i:.2f}' for i in x]) + ']'


def main():

    # 通过动作通信发送已输入的姿态名称指令（内部函数）
    def send_pose():
        # 显示所有已注册的姿态
        for n, j in goals.items():
            print(f'{n:8}{list_to_str(j)}')
        while True:
            kb.set_normal_term()  # 恢复标准键盘输入
            name = input('请输入目标名称: ')
            kb.set_term()  # 恢复非阻塞键盘输入
            if name == '':
                print('已取消')
                return None
            elif name not in goals:
                print(f'{name} 未注册')
            else:
                break
        print('正在发送目标并等待结果…')
        dt = 3.0
        r = commander.send_goal_joint(goals[name], dt)
        print(f'r.result.error_code: {r.result.error_code}')
        # 返回字典中元素的副本，而非引用
        return goals[name].copy()

    # 为当前姿态命名并注册（内部函数）
    def register_pose():
        print('正在注册当前姿态')
        while True:
            kb.set_normal_term()  # 恢复标准键盘输入
            name = input('请输入新姿态的名称: ')
            kb.set_term()  # 恢复非阻塞键盘输入
            if name == '':
                print('已取消')
                return
            if name not in goals:
                break
            print(f'{name} 已存在')
        goals[name] = joint.copy()
        print(f'已注册 {name}: {list_to_str(goals[name])}')

    # 初始化 ROS 客户端
    rclpy.init()

    # 创建节点实例
    commander = Commander()

    # 在另一个线程中运行 rclpy.spin()
    thread = threading.Thread(target=rclpy.spin, args=(commander,))
    threading.excepthook = lambda x: ()
    thread.start()

    # 发布第一条指令前稍作等待
    time.sleep(1.0)

    # 存储姿态名称与对应关节值的字典
    goals = {}
    goals['zeros'] = [0, 0, 0, 0]
    goals['ones'] = [1, 1, 1, 1]
    goals['home'] = [0.0, -1.16, -2.01, -0.73]
    goals['carry'] = [-0.00, -1.37, -2.52, 1.17]

    # 缓慢移动到初始姿态
    joint = [0.0, 0.0, 0.0, 0.0]
    gripper = 0
    dt = 5
    commander.publish_joint(joint, dt)
    commander.publish_gripper(gripper, dt)

    # 创建键盘读取类的实例
    kb = KBHit()

    print('按 1, 2, 3, 4, 5, 6, 7, 8, 9, 0 键来移动关节')
    print('按空格键回到起立状态')
    print('p 键：输入姿态名称并通过动作通信发送指令')
    print('r 键：为当前姿态命名并注册')
    print('按 Esc 键退出')

    # 捕获 KeyboardInterrupt 以避免 Ctrl+C 报错
    try:
        while True:
            time.sleep(0.01)
            # 是否有按键被按下？
            if kb.kbhit():
                c = kb.getch()
                # 保存修改前的值
                joint_prev = joint.copy()
                gripper_prev = gripper

                # 与目标关节值一起发送的目标时间
                dt = 0.2

                # 根据按下的键进行分支处理
                if c == '1':
                    joint[0] -= 0.1
                elif c == '2':
                    joint[0] += 0.1
                elif c == '3':
                    joint[1] -= 0.1
                elif c == '4':
                    joint[1] += 0.1
                elif c == '5':
                    joint[2] -= 0.1
                elif c == '6':
                    joint[2] += 0.1
                elif c == '7':
                    joint[3] -= 0.1
                elif c == '8':
                    joint[3] += 0.1
                elif c == '9':
                    gripper -= 0.1
                elif c == '0':
                    gripper += 0.1
                elif c == ' ':  # 空格键
                    joint = [0.0, 0.0, 0.0, 0.0]
                    gripper = 0
                    dt = 1.0
                elif ord(c) == 27:  # Esc 键
                    break
                elif c == 'p':
                    j = send_pose()
                    if j is not None:
                        joint = j
                elif c == 'r':
                    register_pose()

                # 将指令值限制在有效范围内
                if not all(joint_in_range(joint)):
                    print('关节指令值超出范围')
                    joint = joint_prev.copy()
                if not gripper_in_range(gripper):
                    print('夹爪指令值超出范围')
                    gripper = gripper_prev

                # 如果有变化则发布
                publish = False
                if joint != joint_prev:
                    print(f'joint: {list_to_str(joint)}')
                    commander.publish_joint(joint, dt)
                    publish = True
                if gripper != gripper_prev:
                    print(f'gripper: {gripper:.2f}')
                    commander.publish_gripper(gripper, dt)
                    publish = True
                # 如果已发布，则等待设定的时间
                if publish:
                    time.sleep(dt)
    except KeyboardInterrupt:
        thread.join()
    else:
        print('结束')
        # 缓慢移动到结束姿态
        joint = [0.0, 0.0, 0.0, 0.0]
        gripper = 0
        dt = 5
        commander.publish_joint(joint, dt)
        commander.publish_gripper(gripper, dt)

    rclpy.try_shutdown()