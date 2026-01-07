import sys
import rclpy                        
from rclpy.node import Node          
from geometry_msgs.msg import Twist  # 导入 Twist 消息类型


class HappyTeleop(Node):  # 通过键盘操作发布速度指令的类
    def __init__(self):   # 构造函数
        super().__init__('happy_teleop_node')        
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.vel = Twist()  # 创建 Twist 消息类型实例
        self.vel.linear.x = 0.0
        self.vel.angular.z = 0.0

    def timer_callback(self):  # 定时器回调函数
        key = input('请输入 f, b, r, l, s 后按 Enter <<')  # 获取按键
        # 根据按键值调整线速度或角速度
        if key == 'f':
            self.vel.linear.x += 0.1
        elif key == 'b':
            self.vel.linear.x -= 0.1
        elif key == 'l':
            self.vel.angular.z += 0.1
        elif key == 'r':
            self.vel.angular.z -= 0.1
        elif key == 's':
            self.vel.linear.x = 0.0
            self.vel.angular.z = 0.0
        else:
            print('输入的按键无效。')
        self.publisher.publish(self.vel)  # 发布速度指令消息
        self.get_logger().info(f'线速度={self.vel.linear.x} 角速度={self.vel.angular.z}')


def main():  # 主函数
    rclpy.init()
    node = HappyTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('检测到 Ctrl+C。')
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        rclpy.try_shutdown()    