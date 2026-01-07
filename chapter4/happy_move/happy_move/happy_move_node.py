import math
import sys
import rclpy
import tf_transformations
from rclpy.node import Node   
from rclpy.executors import ExternalShutdownException    
from geometry_msgs.msg import Twist  # 导入 Twist 消息类型
from nav_msgs.msg import Odometry    # 导入 Odometry 消息类型
from tf_transformations import euler_from_quaternion 


class HappyMove(Node):  # 简单的移动类
    def __init__(self):   # 构造函数
        super().__init__('happy_move_node')        
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub = self.create_subscription(Odometry, 'odom', self.odom_cb, 10)   
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.x, self.y, self.yaw = 0.0, 0.0, 0.0
        self.x0, self.y0, self.yaw0 = 0.0, 0.0, 0.0
        self.vel = Twist()  # 创建 Twist 消息类型实例
        self.set_vel(0.0, 0.0)  # 初始化速度
 
    def get_pose(self, msg):      # 获取位姿
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q_x = msg.pose.pose.orientation.x
        q_y = msg.pose.pose.orientation.y
        q_z = msg.pose.pose.orientation.z
        q_w = msg.pose.pose.orientation.w
        (roll, pitch, yaw) = tf_transformations.euler_from_quaternion(
            (q_x, q_y, q_z, q_w))
        return x, y, yaw
  
    def odom_cb(self, msg):         # 里程计回调函数
        self.x, self.y, self.yaw = self.get_pose(msg)
        self.get_logger().info(
            f'x={self.x: .2f} y={self.y: .2f}[m] yaw={self.yaw: .2f}[rad/s]')     
    
    def set_vel(self, linear, angular):  # 设置速度
        self.vel.linear.x = linear   # [m/s]
        self.vel.angular.z = angular  # [rad/s]    
    
    def move_distance(self, dist):  # 移动指定距离 dist
        error = 0.05  # 容许误差 [m] 
        diff = dist - math.sqrt((self.x-self.x0)**2 + (self.y-self.y0)**2) 
        if math.fabs(diff) > error:
            self.set_vel(0.25, 0.0)
            return False
        else:
            self.set_vel(0.0, 0.0)
            return True

    def rotate_angle(self, angle):  # 旋转指定角度 angle
        # 此方法有误，请参考 move_distance 完善。
        self.set_vel(0.0, 0.25)
        return False

    def timer_callback(self):  # 定时器回调函数
        self.pub.publish(self.vel)  # 发布速度指令消息 
        
    def happy_move(self,  distance, angle):  # 简单的状态机
        state = 0
        while rclpy.ok():
            if state == 0:
                if self.move_distance(distance):
                    state = 1
            elif state == 1:                
                if self.rotate_angle(angle):
                    break
            else:
                print('错误状态')
            rclpy.spin_once(self)


def main(args=None):  # 主函数
    rclpy.init(args=args)
    node = HappyMove()

    try:
        node.happy_move(2.0, math.pi/2)
    except KeyboardInterrupt:
        print('检测到 Ctrl+C。')     
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        rclpy.try_shutdown()