import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from message_filters import Subscriber, ApproximateTimeSynchronizer
from tf2_ros import TransformBroadcaster

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

from ultralytics import YOLO


class ObjectDetectionTF(Node):

    def __init__(self):
        super().__init__('object_detection_tf')

        self.target_name = 'cup'  # 要搜索的物体名称
        self.frame_id = 'target'  # 将要广播的 TF 坐标系名称

        # 使用 message_filters 同时订阅三个话题并进行时间同步处理
        self.callback_group = ReentrantCallbackGroup()   # 支持回调函数并发执行
        self.sub_info = Subscriber(
            self, CameraInfo, '/camera/camera/aligned_depth_to_color/camera_info',
            callback_group=self.callback_group)
        self.sub_color = Subscriber(
            self, Image, '/camera/camera/color/image_raw',
            callback_group=self.callback_group)
        self.sub_depth = Subscriber(
            self, Image, '/camera/camera/aligned_depth_to_color/image_raw',
            callback_group=self.callback_group)
        self.ts = ApproximateTimeSynchronizer(
            [self.sub_info, self.sub_color, self.sub_depth], 10, 0.1)
        self.ts.registerCallback(self.images_callback)
        
        # 用于将检测到的物体位置以 TF 形式广播出去
        self.broadcaster = TransformBroadcaster(self)

        self.detection_model = YOLO("yolov8m.pt")

    def images_callback(self, msg_info, msg_color, msg_depth):
        try:
            img_color = CvBridge().imgmsg_to_cv2(msg_color, 'bgr8')
            img_depth = CvBridge().imgmsg_to_cv2(msg_depth, 'passthrough')
        except CvBridgeError as e:
            self.get_logger().warn(str(e))
            return

        if img_color.shape[0:2] != img_depth.shape[0:2]:
            self.get_logger().warn('彩色图像与深度图像尺寸不一致')
            return

        if img_depth.dtype == np.uint16:
            depth_scale = 1e-3
            img_depth_conversion = True
        elif img_depth.dtype == np.float32:
            depth_scale = 1
            img_depth_conversion = False
        else:
            self.get_logger().warn('不支持该深度图像数据类型')
            return
        
        # 执行物体检测
        boxes = []
        classes = []
        results = self.detection_model(img_color, verbose=False)
        names = results[0].names
        boxes = results[0].boxes
        classes = results[0].boxes.cls
        img_color = results[0].plot()

        cv2.imshow('color', img_color)

        # 检查检测结果中是否包含指定的目标物体
        box = None
        for b, c in zip(boxes, classes):
            if names[int(c)] == self.target_name:
                box = b
                break

        # 若在彩色图像中检测到目标，则从深度图像计算其三维位置
        depth = 0
        (bu1, bu2, bv1, bv2) = (0, 0, 0, 0)
        if box is not None:
            a = 0.5
            bu1, bv1, bu2, bv2 = [int(i) for i in box.xyxy.cpu().numpy()[0]]
            u1 = round((bu1 + bu2) / 2 - (bu2 - bu1) * a / 2)
            u2 = round((bu1 + bu2) / 2 + (bu2 - bu1) * a / 2)
            v1 = round((bv1 + bv2) / 2 - (bv2 - bv1) * a / 2)
            v2 = round((bv1 + bv2) / 2 + (bv2 - bv1) * a / 2)
            u = round((bu1 + bu2) / 2)
            v = round((bv1 + bv2) / 2)
            depth = np.median(img_depth[v1:v2+1, u1:u2+1])
            if depth != 0:
                z = float(depth) * depth_scale
                fx = msg_info.k[0]
                fy = msg_info.k[4]
                cx = msg_info.k[2]
                cy = msg_info.k[5]
                x = z / fx * (u - cx)
                y = z / fy * (v - cy)
                self.get_logger().info(
                    f'{self.target_name} 的位置: ({x:.3f}, {y:.3f}, {z:.3f})')
                # 广播 TF 变换
                ts = TransformStamped()
                ts.header = msg_depth.header
                ts.child_frame_id = self.frame_id
                ts.transform.translation.x = x
                ts.transform.translation.y = y
                ts.transform.translation.z = z
                self.broadcaster.sendTransform(ts)

        # 对深度图像进行可视化增强
        if img_depth_conversion:
            img_depth *= 16
        if depth != 0:  # 当成功检测到目标且获取到有效深度时
            pt1 = (int(bu1), int(bv1))
            pt2 = (int(bu2), int(bv2))
            cv2.rectangle(img_depth, pt1=pt1, pt2=pt2, color=0xffff)

        cv2.imshow('depth', img_depth)
        cv2.waitKey(1)


def main():
    rclpy.init()
    object_detection_tf = ObjectDetectionTF()
    try:
        rclpy.spin(object_detection_tf)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()