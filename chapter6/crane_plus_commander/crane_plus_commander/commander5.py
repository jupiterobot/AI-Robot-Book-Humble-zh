import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import euler_from_quaternion
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time
import threading
from math import sqrt
from crane_plus_commander.kbhit import KBHit
from crane_plus_commander.kinematics import (
    inverse_kinematics, joint_in_range, GRIPPER_MAX, GRIPPER_MIN)


# 将 CRANE+ V2 的手尖定位到通过 tf 帧指定的点的节点
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

        # tf
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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

    def get_frame_position(self, frame_id):
        try:
            when = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                'crane_plus_base',
                frame_id,
                when,
                timeout=Duration(seconds=1.0))
        except TransformException as ex:
            self.get_logger().info(f'{ex}')
            return None
        t = trans.transform.translation
        r = trans.transform.rotation
        roll, pitch, yaw = euler_from_quaternion([r.x, r.y, r.z, r.w])
        return [t.x, t.y, t.z, roll, pitch, yaw]


# 计算以列表表示的两个三维坐标之间的距离
def dist(a, b):
    return sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2 + (a[2] - b[2])**2)

def main():
    # ROS客户端的初始化
    rclpy.init()

    # 节点类的实例
    commander = Commander()

    # 在另一个线程中执行 rclpy.spin()
    thread = threading.Thread(target=rclpy.spin, args=(commander,))
    threading.excepthook = lambda x: ()
    thread.start()

    # 在发布第一个指令前稍作等待
    time.sleep(2.0)

    # 缓慢移动到初始姿态
    joint = [0.0, -1.16, -2.01, -0.73]
    gripper = GRIPPER_MIN
    dt = 5
    commander.publish_joint(joint, dt)
    commander.publish_gripper(gripper, dt)

    # 逆运动学解的类型
    elbow_up = True

    # 键盘读取类的实例
    kb = KBHit()

    # 状态
    INIT = 0
    WAIT = 1
    DONE = 2
    state = INIT

    print('按 r 键重新初始化')
    print('按 Esc 键退出')

    # 为避免 Ctrl+C 导致错误，捕获 KeyboardInterrupt
    try:
        while True:
            time.sleep(0.01)            
            # 是否有按键被按下？
            if kb.kbhit():
                c = kb.getch()
                if c == 'r':
                    print('重新初始化')
                    state = INIT
                elif ord(c) == 27:  # Esc 键
                    break

            position = commander.get_frame_position('target')
            if position is None:
                print('未找到目标帧')
            else:
                xyz_now = position[0:3]
                time_now = time.time()
                if state == INIT:
                    xyz_first = xyz_now
                    time_first = time_now
                    state = WAIT
                elif state == WAIT:
                    if dist(xyz_now, xyz_first) > 0.01:
                        state = INIT
                    elif time_now - time_first > 1.0:
                        state = DONE
                        pitch = 0
                        joint = inverse_kinematics(xyz_now + [pitch], elbow_up)
                        if joint is None:
                            print('逆运动学无解')
                        elif not all(joint_in_range(joint)):
                            print('关节指令值超出范围')
                        else:
                            print(f'关节指令值： [{joint[0]:.2f}, {joint[1]:.2f},',
                                f'{joint[2]:.2f}, {joint[3]:.2f}]')
                            dt = 0.5
                            commander.publish_joint(joint, dt)
                            time.sleep(dt)
    except KeyboardInterrupt:
        thread.join()
    else:
        print('结束')
        # 缓慢移动到结束姿态
        joint = [0.0, 0.0, 0.0, 0.0]
        gripper = GRIPPER_MAX
        dt = 5
        commander.publish_joint(joint, dt)
        commander.publish_gripper(gripper, dt)

    rclpy.try_shutdown()