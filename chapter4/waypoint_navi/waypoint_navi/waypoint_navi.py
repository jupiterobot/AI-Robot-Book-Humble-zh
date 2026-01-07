#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy  
from rclpy.node import Node  
import sys   
import time  
from math import pi  
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped  # 位置和姿态相关消息
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult  # 用于 Nav2 Simple Commander
import tf_transformations  # 用于旋转转换的库

class WayPointNavi(Node):
    def __init__(self):
        super().__init__('waypoint_navi')  # 初始化节点
        self.wp_num = 0                    # 初始化航点编号
        self.init_pose = [-2.0, -0.5, 0.0] # 初始姿态（x, y, yaw）
        self.navigator = BasicNavigator()  # 创建 BasicNavigator 实例
        
    def do_navigation(self):  ### 执行导航的方法 ###
        way_point = [         # 航点列表
            [1.2, -1.5, pi/2], [1.0, 0.5, pi], [-4.0, 0.8, pi/2], [-4.0, 3.9, pi], 
            [-6.5, 4.0, -pi/2], [-6.5, -3.0, pi/2], [999.9, 0.0, 0.0] 
        ]        
        self.set_init_pose()  # 设置初始姿态
        self.navigator.waitUntilNav2Active()            # 等待 Nav2 激活
        while rclpy.ok():     # 导航主循环
            if way_point[self.wp_num][0] == 999.9:      # 检查终止条件
                self.get_logger().info('导航已结束。')
                sys.exit(0)                             # 正常退出程序
            pose_msg = self.to_pose_msg(way_point[self.wp_num])  # 设置当前航点的姿态
            result = self.navigate_to_goal(pose_msg)    # 导航至目标点
            time.sleep(1)  

    def set_init_pose(self):  ### 向 Nav2 设置初始姿态的方法 ###
        init_pose_msg = self.to_pose_msg(self.init_pose)   # 转换为消息类型     
        self.get_logger().info('正在设置初始位置。')
        self.navigator.setInitialPose(init_pose_msg)       # 向 Nav2 设置初始姿态        

    def navigate_to_goal(self, goal_pose):  ### 导航至目标点 ###
        self.get_logger().info(f"前往 WP{self.wp_num + 1}({goal_pose.pose.position.x},{goal_pose.pose.position.y})。")
        self.navigator.goToPose(goal_pose)            # 指定目标并开始导航
        while not self.navigator.isTaskComplete():    # 等待任务完成
            feedback = self.navigator.getFeedback()   # 获取反馈
            if feedback:
                self.get_logger().info(f"剩余距离：{feedback.distance_remaining:.2f}[m]")
                self.get_logger().info(f"耗时：{feedback.navigation_time.sec}[s]") 
                if feedback.navigation_time.sec > 99: # 若导航超时
                    self.navigator.cancelTask()       # 取消任务
            time.sleep(0.5)                           # 反馈获取间隔
        result = self.navigator.getResult()           # 获取结果        
        if result == TaskResult.SUCCEEDED:            # 成功
            self.get_logger().info(f'已到达 WP{self.wp_num + 1}。')
            self.wp_num += 1                          # 进入下一航点
        elif result == TaskResult.CANCELED:           # 被取消
            self.get_logger().info(f"WP{self.wp_num + 1} 已被取消。")
            self.wp_num += 1                          # 跳过该点，进入下一航点
        else:                                         # 失败
            self.get_logger().info(f'WP{self.wp_num + 1} 失败。')
            sys.exit(1)                               # 异常退出        
    
    def to_pose_msg(self, pose):  ### 转换为消息类型的方法 ###
        pose_msg = PoseStamped()               # 设置航点姿态
        pose_msg.header.stamp = self.navigator.get_clock().now().to_msg()  # 当前时间
        pose_msg.header.frame_id = "map"       # 设置坐标系
        pose_msg.pose.position.x = pose[0]     # x 坐标
        pose_msg.pose.position.y = pose[1]     # y 坐标
        q = tf_transformations.quaternion_from_euler(0, 0, pose[2])  # 角度转换
        pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, \
        pose_msg.pose.orientation.z, pose_msg.pose.orientation.w = q
        return pose_msg


def main(args=None):
    rclpy.init(args=args)
    waypoint_navi = WayPointNavi()
    waypoint_navi.do_navigation()
    rclpy.spin(waypoint_navi)
    waypoint_navi.destroy_node()  
    rclpy.shutdown()