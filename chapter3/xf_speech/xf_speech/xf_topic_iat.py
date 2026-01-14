#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pathlib import Path
import sys
import time

pkg_path = str(Path(__file__).resolve().parents[1])
sys.path.insert(0, pkg_path + "/xf_speech")
import xf_iat


class XfPublishC(Node):
    def __init__(self):
        super().__init__('xf_iat_publish_node')
        self.get_logger().info("Sleeping……")  # 保持初始输出
        
        self.publisher_ = self.create_publisher(String, 'voiceWords', 1)
        self.subscription = self.create_subscription(
            String,
            'voiceWakeup',
            self.wakeup_callback,
            1)
        
        self.is_processing = False

    def wakeup_callback(self, msg):
        if self.is_processing:
            self.get_logger().info("已有处理进程在进行中，忽略本次唤醒")
            return
    
        else:
            self.is_processing = True
            self.voice_processing()

    def voice_processing(self):
        # 阶段1：准备录音
        self.get_logger().info("publishing and latching message for 3.0 seconds")
        time.sleep(3)
        
        # 阶段2：开始录音识别
        self.get_logger().info("Wakeup…… \nSpeak in 10 seconds\nStart Listening……")
        
        result = xf_iat.iat_main(input_t=10)
        
        # 阶段3：处理识别结果
        self.get_logger().info(result)

        self.get_logger().info("Speaking done\nNot started or already stopped.\n10 sec passed")
        
        msg = String()

        msg.data = result

        self.publisher_.publish(msg)
        
        self.is_processing = False
        self.get_logger().info("Sleeping……")

def main(args=None):
    rclpy.init(args=args)
    node = XfPublishC()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("用户主动终止节点")
    finally:
        try:
            node.destroy_node()
        finally:
            rclpy.shutdown()

if __name__ == '__main__':
    main()
