import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tf_transformations as tf_trans 
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from tf_transformations import euler_from_quaternion

class AMCLSubscriber(Node):                                                                               
    def __init__(self):        # 构造函数
        super().__init__('amcl_subscriber_node')                                                          
        self.create_subscription(PoseWithCovarianceStamped,
                                                 'amcl_pose', self.amcl_cb, 10)                  
        self.create_subscription(Odometry,'odom',self.odom_cb, 10)                  
        self.create_timer(0.1, self.timer_cb)  
        self.last_amcl_msg = Pose()
        self.last_odom_msg = Pose()                                                                                      

    def get_pose(self,msg):      # 获取位姿
        pos = msg.position
        q = msg.orientation
        roll, pitch, yaw = tf_trans.euler_from_quaternion((q.x, q.y, q.z, q.w))
        return pos.x, pos.y, yaw

    def amcl_cb(self, msg):         # AMCL 回调函数
        self.last_amcl_msg = msg.pose.pose
  
    def odom_cb(self, msg):         # ODOM 回调函数
        self.last_odom_msg = msg.pose.pose
 
    def timer_cb(self):             # 定时器回调函数
        for name, msg in [('AMCL',self.last_amcl_msg),('ODOM',self.last_odom_msg)]:
            x, y, yaw = self.get_pose(msg)        
            if msg:
                self.get_logger().info(f'{name}: x={x:.2f} y={y:.2f} [m] theta={yaw:.2f}[rad/s]')
            else:
                self.get_logger().info(f'No {name} received yet.')
        
def main(args=None):                                                                                    
    rclpy.init(args=args)                                                                               
    amcl_subscriber = AMCLSubscriber()                                                                      
    rclpy.spin(amcl_subscriber)                                                                           
    amcl_subscriber.destory_node()                                                                        
    rclpy.shutdown()                                                                                    