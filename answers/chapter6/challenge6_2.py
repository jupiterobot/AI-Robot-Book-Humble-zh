import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time
import threading
from crane_plus_commander.kbhit import KBHit
from crane_plus_commander.kinematics6_2 import (
    forward_kinematics, from_gripper_ratio, gripper_in_range,
    inverse_kinematics, joint_in_range, to_gripper_ratio, collides_with_table)


# 用于向 CRANE+ V2 发布指令的节点
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


def main():
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

    # 缓慢移动到初始姿态
    joint = [0.0, -1.16, -2.01, -0.73]
    gripper = 0
    dt = 5
    commander.publish_joint(joint, dt)
    commander.publish_gripper(gripper, dt)

    # 逆运动学解的类型（肘部朝上）
    elbow_up = True

    # 创建键盘读取类的实例
    kb = KBHit()

    print('按 1, 2, 3, 4, 5, 6, 7, 8, 9, 0 键来移动关节')
    print('按 a, z, s, x, d, c, f, v, g, b 键来移动手尖')
    print('按 e 键切换逆运动学的解')
    print('按空格键回到起立状态')
    print('按 Esc 键退出')

    # 捕获 KeyboardInterrupt 以避免 Ctrl+C 报错
    try:
        while True:
            time.sleep(0.01)
            # 是否有按键被按下？
            if kb.kbhit():
                c = kb.getch()

                # 正运动学计算
                [x, y, z, pitch] = forward_kinematics(joint)
                ratio = to_gripper_ratio(gripper)
                # 保存修改前的值
                joint_prev = joint.copy()
                gripper_prev = gripper
                elbow_up_prev = elbow_up

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
                elif c == 'a':
                    x += 0.01
                elif c == 'z':
                    x -= 0.01
                elif c == 's':
                    y += 0.01
                elif c == 'x':
                    y -= 0.01
                elif c == 'd':
                    z += 0.01
                elif c == 'c':
                    z -= 0.01
                elif c == 'f':
                    pitch += 0.1
                elif c == 'v':
                    pitch -= 0.1
                elif c == 'g':
                    ratio += 0.1
                elif c == 'b':
                    ratio -= 0.1
                elif c == 'A':
                    x += 0.05
                elif c == 'Z':
                    x -= 0.05
                elif c == 'S':
                    y += 0.05
                elif c == 'X':
                    y -= 0.05
                elif c == 'D':
                    z += 0.05
                elif c == 'C':
                    z -= 0.05
                elif c == 'F':
                    pitch += 0.5
                elif c == 'V':
                    pitch -= 0.5
                elif c == 'G':
                    ratio += 0.5
                elif c == 'B':
                    ratio -= 0.5
                elif c == 'e':
                    elbow_up = not elbow_up
                    print(f'elbow_up: {elbow_up}')
                    dt = 3.0
                elif c == ' ':  # 空格键
                    joint = [0.0, 0.0, 0.0, 0.0]
                    gripper = 0
                    dt = 1.0
                elif ord(c) == 27:  # Esc 键
                    break

                # 逆运动学计算
                if c in 'azsxdcfveAZSXDCFV':
                    joint = inverse_kinematics([x, y, z, pitch], elbow_up)
                    if joint is None:
                        print('逆运动学无解')
                        joint = joint_prev.copy()
                elif c in 'gbGB':
                    gripper = from_gripper_ratio(ratio)

                # 将指令值限制在有效范围内
                if not all(joint_in_range(joint)):
                    print('关节指令值超出范围')
                    joint = joint_prev.copy()
                    elbow_up = elbow_up_prev
                if not gripper_in_range(gripper):
                    print('夹爪指令值超出范围')
                    gripper = gripper_prev

                # 检查手尖是否可能与桌面发生干涉
                if collides_with_table(joint):
                    print('手尖可能与桌面发生干涉')
                    joint = joint_prev.copy()
                    elbow_up = elbow_up_prev

                # 如果有变化则发布
                publish = False
                if joint != joint_prev:
                    print((f'joint: [{joint[0]:.2f}, {joint[1]:.2f}, '
                           f'{joint[2]:.2f}, {joint[3]:.2f}]'))
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