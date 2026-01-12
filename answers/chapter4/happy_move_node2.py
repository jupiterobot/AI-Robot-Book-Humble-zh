import math
import sys
import time
import rclpy
import tf_transformations
from rclpy.node import Node   
from rclpy.executors import ExternalShutdownException   
from geometry_msgs.msg import Twist  # 导入 Twist 消息类型
from nav_msgs.msg import Odometry    # 导入 Odometry 消息类型
from tf_transformations import euler_from_quaternion 


class HappyMove(Node):  # 简单移动类
    def __init__(self):   # 构造函数
        super().__init__('happy_move_node2')        
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub = self.create_subscription(Odometry, 'odom', self.odom_cb, 10)   
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.x, self.y, self.yaw = 0.0, 0.0, 0.0
        self.x0, self.y0, self.yaw0 = 0.0, 0.0, 0.0
        self.vel = Twist()  # 创建 Twist 消息实例
        self.set_vel(0.0, 0.0)  # 初始化速度
        self.start_time = time.time()
        self.state = 0
        self.state_old = 0
 
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
        #self.get_logger().info(
        #    f'x={self.x: .2f} y={self.y: .2f}[m] yaw={self.yaw: .2f}[rad/s]')     
    
    def set_vel(self, linear, angular):  # 设置速度
        self.vel.linear.x = linear   # [m/s]
        self.vel.angular.z = angular  # [rad/s]    
    
    def move_distance(self, dist):  # 移动指定距离 dist
        error = 0.05  # 允许误差 [m] 
        diff = dist - math.sqrt((self.x-self.x0)**2 + (self.y-self.y0)**2) 
        if math.fabs(diff) > error:
            self.set_vel(0.25, 0.0)
            return False
        else:
            self.set_vel(0.0, 0.0)
            return True

    def rotate_angle(self, angle):  # Challenge 4.1
        error = 0.05  # 允许误差 [rad]
        diff = self.yaw - self.yaw0
        # 归一化：使 diff 落在 -π 到 π 之间
        while diff <= -math.pi:
            diff += 2 * math.pi
        while diff > math.pi:
            diff -= 2 * math.pi
        target_angle = angle

        # 归一化
        while target_angle <= -math.pi:
            target_angle += 2 * math.pi
        while target_angle > math.pi:
            target_angle -= 2 * math.pi
        angle_error = abs(target_angle - diff)

        # 归一化
        if angle_error > math.pi:
            angle_error = 2 * math.pi - angle_error
        if angle_error > error:
            if target_angle > diff:
                self.set_vel(0.0, 0.25)
            else:
                self.set_vel(0.0, -0.25)
            return False
        else:
            self.set_vel(0.0, 0.0)
            rclpy.spin_once(self)
            return True
     
    def move_time(self, linear_vel, angular_vel, duration): # Challenge 4.2        
        self.set_vel(linear_vel, angular_vel)
        while rclpy.ok():
            current_time = time.time()
            diff_time = current_time - self.start_time
            if diff_time >= duration:
                self.set_vel(0.0, 0.0)
                rclpy.spin_once(self)
                return True            
            return False        
 
    def timer_callback(self):  # 定时器回调函数
        self.pub.publish(self.vel)  # 发布速度指令消息 

    def set_init_pos(self):
        self.x0 = self.x
        self.y0 = self.y

    def set_init_yaw(self):
        self.yaw0 = self.yaw

    def draw_square(self, x):
        linear_vel = 0.25
        angular_vel = 0.3
        for num in range(4):
            rclpy.spin_once(self)
            self.set_init_pos()
            
            while rclpy.ok():
                if not self.move_distance(x):
                    self.set_vel(linear_vel, 0.0)
                    rclpy.spin_once(self, timeout_sec=0.01)
                else:
                    self.set_vel(0.0, 0.0)
                    rclpy.spin_once(self, timeout_sec=0.01)
                    break;                
            rclpy.spin_once(self)
            self.set_init_yaw()
            while rclpy.ok():            
                if not self.rotate_angle(math.pi/2):
                    self.set_vel(0.0, angular_vel)
                    rclpy.spin_once(self, timeout_sec=0.01)
                else:
                    self.set_vel(0.0, 0.0)
                    rclpy.spin_once(self, timeout_sec=0.01)
                    break;
            num += 1                
        return True

    def draw_circle(self, r):
        linear_speed = 0.25
        angular_speed = linear_speed / r
        duration = 0.01
        steps = int(2 * math.pi / (angular_speed * duration))            
        for _ in range(steps):
            self.set_vel(linear_speed, angular_speed)
            rclpy.spin_once(self)
            time.sleep(0.01)               
        self.set_vel(0.0, 0.0)
        rclpy.spin_once(self)
        return True            
        
    def happy_move2(self, distance, angle, linear_vel, angular_vel, duration):  # 简单状态机
        self.state = 0  
        while rclpy.ok():
            if self.state == 0: # Challenge 4.1       
                print(f'*** state {self.state}: Challenge 4.1')
                if self.rotate_angle(angle):
                    break                    
            elif self.state == 1: # Challenge 4.2
                print(f'*** state {self.state}: Challenge 4.2')
                if self.state != self.state_old:
                    self.start_time = time.time()
                    self.state_old = self.state
                if self.move_time(linear_vel, angular_vel, duration):
                    break                    
            elif self.state == 2:  # Challenge 4.3.1: 画正方形
                print(f'*** state {self.state}: Challenge 4.3.1')
                length = 1.0 # 边长
                if self.draw_square(length):
                    break                
            elif self.state == 3:  # Challenge 4.3.2: 画圆
                print(f'*** state {self.state}: Challenge 4.3.2')
                radius = 1.0 # 半径
                if self.draw_circle(radius):
                    break
            else:
                print(f'错误状态: {self.state}')
                sys.exit(1)            
            rclpy.spin_once(self, timeout_sec=0.01)    


def main(args=None):  # 主函数
    rclpy.init(args=args)
    node = HappyMove()

    try:
        node.happy_move2(2.0, math.pi/2, 0.5, math.pi/5, 10.0)
    except KeyboardInterrupt:
        print('检测到 Ctrl+C。')     
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        rclpy.try_shutdown()