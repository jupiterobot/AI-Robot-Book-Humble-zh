import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time
import threading
from crane_plus_commander.kbhit import KBHit
from crane_plus_commander.kinematics import gripper_in_range, joint_in_range


# 用于向 CRANE+ V2 发送关节指令的 ROS 2 节点
class Commander(Node):

    def __init__(self):
        super().__init__('commander')
        # 定义机械臂的四个关节名称
        self.joint_names = [
            'crane_plus_joint1',
            'crane_plus_joint2',
            'crane_plus_joint3',
            'crane_plus_joint4'
        ]
        # 创建发布机械臂关节轨迹的发布者
        self.publisher_joint = self.create_publisher(
            JointTrajectory,
            'crane_plus_arm_controller/joint_trajectory', 10)
        # 创建发布夹爪（gripper）轨迹的发布者
        self.publisher_gripper = self.create_publisher(
            JointTrajectory,
            'crane_plus_gripper_controller/joint_trajectory', 10)

    def publish_joint(self, q, time_sec):
        """发布机械臂关节目标位置"""
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        msg.points = [JointTrajectoryPoint()]
        msg.points[0].positions = [
            float(q[0]), float(q[1]), float(q[2]), float(q[3])
        ]
        # 设置从开始到目标点的时间（支持小数秒）
        msg.points[0].time_from_start = Duration(
            seconds=int(time_sec),
            nanoseconds=(time_sec - int(time_sec)) * 1e9
        ).to_msg()
        self.publisher_joint.publish(msg)

    def publish_gripper(self, gripper, time_sec):
        """发布夹爪目标开合角度"""
        msg = JointTrajectory()
        msg.joint_names = ['crane_plus_joint_hand']
        msg.points = [JointTrajectoryPoint()]
        msg.points[0].positions = [float(gripper)]
        msg.points[0].time_from_start = Duration(
            seconds=int(time_sec),
            nanoseconds=(time_sec - int(time_sec)) * 1e9
        ).to_msg()
        self.publisher_gripper.publish(msg)


def main():
    # 初始化 ROS 2 客户端
    rclpy.init()

    # 创建 Commander 节点实例
    commander = Commander()

    # 在独立线程中运行 rclpy.spin()，避免阻塞主循环
    thread = threading.Thread(target=rclpy.spin, args=(commander,))
    threading.excepthook = lambda x: ()  # 忽略线程异常
    thread.start()

    # 等待 1 秒，确保节点和控制器已就绪
    time.sleep(1.0)

    # 缓慢移动到初始姿态（全零位姿）
    joint = [0.0, 0.0, 0.0, 0.0]
    gripper = 0.0
    dt = 5.0  # 用 5 秒缓慢移动
    commander.publish_joint(joint, dt)
    commander.publish_gripper(gripper, dt)

    # 创建键盘监听对象
    kb = KBHit()

    print('按 1～0 键控制各关节：')
    print('  1/2 → 关节1（左右旋转）')
    print('  3/4 → 关节2')
    print('  5/6 → 关节3')
    print('  7/8 → 关节4')
    print('  9/0 → 夹爪（闭合/张开）')
    print('按 Shift + 数字键可进行大幅调整（±0.5）')
    print('按空格键返回起立初始姿态')
    print('按 Esc 键退出程序')

    try:
        while True:
            time.sleep(0.01)  # 减少 CPU 占用

            if kb.kbhit():  # 检测是否有按键按下
                c = kb.getch()
                # 保存当前值，以便在越界时回滚
                joint_prev = joint.copy()
                gripper_prev = gripper

                dt = 0.2  # 默认运动时间（快速响应）

                # 根据按键调整关节或夹爪目标值
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
                # 支持 Shift + 数字（大幅调整 ±0.5）
                elif c == '!':   # Shift+1
                    joint[0] -= 0.5
                elif c == '"':   # Shift+2
                    joint[0] += 0.5
                elif c == '#':   # Shift+3
                    joint[1] -= 0.5
                elif c == '$':   # Shift+4
                    joint[1] += 0.5
                elif c == '%':   # Shift+5
                    joint[2] -= 0.5
                elif c == '&':   # Shift+6
                    joint[2] += 0.5
                elif c == "'":   # Shift+7
                    joint[3] -= 0.5
                elif c == '(':   # Shift+8
                    joint[3] += 0.5
                elif c == ')':   # Shift+9
                    gripper -= 0.5
                elif c == '～':   # Shift+0
                    gripper += 0.5
                elif c == ' ':   # 空格键：回到初始姿态
                    joint = [0.0, 0.0, 0.0, 0.0]
                    gripper = 0.0
                    dt = 1.0  # 回到初始姿态用 1 秒
                elif ord(c) == 27:  # Esc 键
                    break

                # 检查指令是否在安全范围内
                if not all(joint_in_range(joint)):
                    print('关节指令值超出允许范围！')
                    joint = joint_prev.copy()  # 回滚
                if not gripper_in_range(gripper):
                    print('夹爪指令值超出允许范围！')
                    gripper = gripper_prev      # 回滚

                # 如果有变化，则发布新指令
                publish = False
                if joint != joint_prev:
                    print(f'关节目标: [{joint[0]:.2f}, {joint[1]:.2f}, '
                          f'{joint[2]:.2f}, {joint[3]:.2f}]')
                    commander.publish_joint(joint, dt)
                    publish = True
                if gripper != gripper_prev:
                    print(f'夹爪目标: {gripper:.2f}')
                    commander.publish_gripper(gripper, dt)
                    publish = True

                # 若已发布指令，则等待运动完成（休眠 dt 秒）
                if publish:
                    time.sleep(dt)

    except KeyboardInterrupt:
        print('\n收到 Ctrl+C，正在退出...')
        thread.join()
    else:
        print('正常退出')
        # 平稳返回初始姿态后关闭
        joint = [0.0, 0.0, 0.0, 0.0]
        gripper = 0.0
        dt = 5.0
        commander.publish_joint(joint, dt)
        commander.publish_gripper(gripper, dt)

    # 尝试安全关闭 ROS
    rclpy.try_shutdown()