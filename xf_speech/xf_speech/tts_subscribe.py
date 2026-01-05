#! /usr/bin/env python3
# -*- coding:utf-8 -*-

import rclpy
from std_msgs.msg import String
from pathlib import Path
import sys

# 获取包路径和脚本路径，并添加到系统路径中
pkg_path = str(Path(__file__).resolve().parents[1]) # 获取pkg路径
sys.path.insert(0, pkg_path + "/xf_speech") # 获取pkg/scripts路径
import xf_tts  # 导入xf_tts处理函数

def doMsg(msg):
    print("\nI heard:%s" % msg.data)
    xf_tts.tts_main(msg.data)

def main(args=None):
    # 初始化ROS节点: 命名(唯一)
    rclpy.init(args=args)
    node = rclpy.create_node('tts_subscribe_node')
    node.get_logger().info(
        "\n##############################################################\
        \n # 语音合成 tts(text to speech) \
        \n##############################################################")

    # 实例化订阅者对象
    sub = node.create_subscription(String, 'voiceWords', doMsg, 10)

    try:
        # 循环调用回调函数
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 确保关闭节点
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()