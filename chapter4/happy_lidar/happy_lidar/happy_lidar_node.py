import math
import os
import sys
import rclpy
import rospkg
import time
import tf_transformations
from rclpy.node import Node   
from rclpy.executors import ExternalShutdownException    
from geometry_msgs.msg import Twist, Pose, Point  # 导入 Twist 消息类型
from nav_msgs.msg import Odometry                 # 导入 Odometry 消息类型
from sensor_msgs.msg import LaserScan             # 导入 LaserScan 消息类型
from tf_transformations import euler_from_quaternion 
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity, DeleteEntity


class HappyLidar(Node):  # 简单的 LiDAR 类
    def __init__(self):   # 构造函数
        super().__init__('happy_lidar_node')   
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10) 
        self.sub = self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
        self.x, self.y, self.yaw = 0.0, 0.0, 0.0   
        self.x0, self.y0, self.yaw0 = 0.0, 0.0, 0.0
        self.vel = Twist()      # 创建 Twist 消息类型实例
        self.set_vel(0.0, 0.0)  # 初始化速度
        # 为使用 LiDAR 而添加
        self.sub = self.create_subscription(LaserScan, 'scan', self.lidar_cb, 10)   
        self.scan = LaserScan()  # 创建 LaserScan 消息类型实例
        self.scan.ranges = [-99.9] * 360 # 用不可能的值初始化，以区别于实际获取的数据
        
    def lidar_cb(self, msg):  # LiDAR 回调函数
        self.scan = msg
        for i in range(len(msg.ranges)):
            if msg.ranges[i] < msg.range_min or msg.ranges[i] > msg.range_max:
                pass # 需要处理超出测量范围的数据，此处跳过。
            else:
                self.scan.ranges[i] = msg.ranges[i] 
    
    def happy_lidar(self): # 门打开后前进的方法
        steps = 0
        self.load_gazebo_models()  # 加载门模型
        time.sleep(3)              # 等待门在仿真器中加载完成
        self.set_vel(0.0, 0.0)     # 停止  
        rclpy.spin_once(self)      # 调用回调函数  

        while rclpy.ok():
            print(f'step={steps}')         
            if steps == 100: 
                self.delete_gazebo_models()   # 删除门模型（模拟开门） 
            dist = 0.5                      # 启动距离   
            if self.scan.ranges[0] > dist:  # 门打开时的处理
                self.set_vel(0.2, 0.0)        # 前进 
            else:
                self.set_vel(0.0, 0.0)        # 停止
                
            rclpy.spin_once(self)
            # self.print_lidar_info() # 如需显示 scan 话题的值，请取消注释
            print(f'r[{  0}]={self.scan.ranges[0]}')     # 前
            print(f'r[{ 90}]={self.scan.ranges[90]}')    # 左
            print(f'r[{180}]={self.scan.ranges[180]}')   # 后
            print(f'r[{270}]={self.scan.ranges[270]}')   # 右
  
            time.sleep(0.1)  # 0.1 [s]
            steps += 1
    
    def print_lidar_info(self):
        # 仿真中的 LiDAR (HILS-LFCD LDS) 可测量 360° 全方位，测量角度范围为 -180° 到 180°。
        # ROS 使用右手坐标系：前进方向为 x 轴，左侧为 y 轴，上方为 z 轴（逆时针方向为正）。
        if len(self.scan.ranges) == 0:
            return
        self.get_logger().info(
            f'Angle [rad] min={self.scan.angle_min:3f} max={self.scan.angle_max:3f}')
        self.get_logger().info(
            f'Angle2 [rad] min={self.angle_min:3f} max={self.angle_max:3f}')
        # ROS 中的角度单位为弧度 [rad]，此处使用 math.degrees 函数将弧度转换为角度。
        self.get_logger().info(
            f'Angle [deg] increment={math.degrees(self.scan.angle_increment):.3f}')
        self.get_logger().info(
            f'Range [m] min={self.scan.range_min:.3f} max={self.scan.range_max:.3f}')
        print(f'len={len(self.scan.ranges)}')
        print(f'r[{0}]={self.scan.ranges[0]} ')
        print(f'r[{90}]={self.scan.ranges[90]} ')
        print(f'r[{180}]={self.scan.ranges[180]} ')
        print(f'r[{270}]={self.scan.ranges[270]} ')
  
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
    
    #  加载 Gazebo 用的门 3D 模型
    def load_gazebo_models(self):
        door_reference_frame='world'
        door_pose = Pose(position=Point(x=1.1, y=-0.2, z=0.5))

        client = self.create_client(SpawnEntity, "/spawn_entity")

        if not client.service_is_ready():
            client.wait_for_service()
            
        home_dir = os.environ['HOME']    
        sdf_file_path = home_dir + '/airobot_ws/src/chapter4/happy_lidar/models/door/model.sdf'

        # 设置请求数据
        request = SpawnEntity.Request()
        request.name = 'door'
        request.xml = open(sdf_file_path, 'r').read()
        request.robot_namespace = ''
        request.reference_frame = 'world'
        request.initial_pose.position.x =  1.1
        request.initial_pose.position.y = -0.2
        request.initial_pose.position.z =  0.5
        
        self.get_logger().info("正在向 `/spawn_entity` 发送服务请求...")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            print(f'response:{future.result()}')
        else:
            raise RuntimeError(f'异常:{future.exception()}')

    #  删除门的 3D 模型
    def delete_gazebo_models(self):
        print('delete_gazebo_models')
        client = self.create_client(DeleteEntity, '/delete_entity')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('服务不可用，等待中...')
        # 设置请求数据
        request = DeleteEntity.Request()
        request.name = 'door'      
     
        self.get_logger().info("正在向 `/delete_entity` 发送服务请求...")
        future =  client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            print(f'response:{future.result()}')
        else:
            raise RuntimeError(f'异常:{future.exception()}')
  

def main():  # 主函数
    rclpy.init()
    node = HappyLidar()
    try:
        node.happy_lidar()
    except KeyboardInterrupt:
        print('检测到 Ctrl+C。')     
    finally:
        rclpy.shutdown()