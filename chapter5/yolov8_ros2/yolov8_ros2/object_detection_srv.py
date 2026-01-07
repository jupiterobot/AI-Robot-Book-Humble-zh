import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from message_filters import Subscriber, ApproximateTimeSynchronizer
from tf2_ros import TransformBroadcaster

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

from ultralytics import YOLO

import threading
import queue
import time

from airobot_interfaces.srv import StringCommand


class ObjectDetectionService(Node):

    def __init__(self):
        super().__init__('object_detection_service')

        self.running = False  # 物体检测的处理标志
        self.target_name = 'cup'  # 要查找的物体名称
        self.frame_id = 'target'  # 广播的 tf 名称
        self.counter = 0  # 计算检测到物体次数的计数器
        self.q_color = queue.Queue()  # 向主线程发送彩色图像的队列
        self.q_depth = queue.Queue()  # 向主线程发送深度图像的队列
        self.lock = threading.Lock()  # 两个回调方法间的互斥控制用

        # 使用 message_filters 同时处理三个话题的订阅。
        self.callback_group = ReentrantCallbackGroup()   # 用于回调的并发处理
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

        # 用于将识别到的物体位置以 tf 形式广播的 broadcaster
        self.broadcaster = TransformBroadcaster(self)

        # 接收来自其他节点指令的服务服务器
        self.service = self.create_service(
            StringCommand,
            'vision/command',
            self.command_callback,
            callback_group=self.callback_group
            )
        
        self.detection_model = YOLO('yolov8m.pt')
        # 通过执行一次虚拟图像推理来加载 YOLO 缓存
        dummy_image = np.zeros((640, 480, 3), dtype=np.uint8)
        results = self.detection_model(dummy_image, verbose=False)
        self.names = results[0].names.values()
        threading.excepthook = lambda x: ()

    # StringCommand 的请求到达时调用的方法
    def command_callback(self, request, response):
        self.get_logger().info(f'command: {request.command}')
        if request.command.startswith('find'):
            name = request.command[4:].strip()
            if len(name) == 0:
                response.answer = 'NG name required'
            elif name not in self.names:
                response.answer = 'NG unknown name'
            else:
                with self.lock:  # 开始物体检测
                    self.target_name = name
                    self.running = True
                    self.counter = 0
                time.sleep(3)   # 等待一段时间
                with self.lock:  # 获取物体检测的结果
                    self.running = False
                    counter = self.counter
                    print(counter)
                if counter >= 2:  # 是否有足够的次数被识别？
                    response.answer = 'OK'
                else:
                    response.answer = 'NG not found'
        elif request.command.startswith('track'):
            name = request.command[5:].strip()
            if len(name) == 0:
                response.answer = 'NG name required'
            elif name not in self.names:
                response.answer = 'NG unknown name'
            else:
                with self.lock:  # 开始物体检测
                    self.target_name = name
                    self.running = True
                response.answer = 'OK'
        elif request.command.startswith('stop'):
            with self.lock:
                self.running = False
            response.answer = 'OK'
        else:
            response.answer = f'NG {request.command} not supported'
        self.get_logger().info(f'answer: {response.answer}')
        return response

    # 当接收到相机信息、彩色图像和深度图像时调用的方法
    def images_callback(self, msg_info, msg_color, msg_depth):
        # 将订阅的图像转换为 OpenCV 格式
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

        # 与服务回调方法共享
        with self.lock:
            target_name = self.target_name
            running = self.running

        # 物体检测
        boxes = []
        classes = []
        if running:
            results = self.detection_model(img_color, verbose=False)
            names = results[0].names
            boxes = results[0].boxes
            classes = results[0].boxes.cls
            img_color = results[0].plot()
            with self.lock:
                self.counter += 1

        self.q_color.put(img_color)  # 向主线程发送彩色图像。

        # 检查检测结果中是否存在指定名称的物体。
        box = None
        for b, c in zip(boxes, classes):
            if names[int(c)] == target_name:
                box = b
                break

        # 若在彩色图像中检测到，则从深度图像计算三维位置。
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
                    f'{target_name} ({x:.3f}, {y:.3f}, {z:.3f})')
                # 发布 tf
                ts = TransformStamped()
                ts.header = msg_depth.header
                ts.child_frame_id = self.frame_id
                ts.transform.translation.x = x
                ts.transform.translation.y = y
                ts.transform.translation.z = z
                self.broadcaster.sendTransform(ts)

                with self.lock:
                    self.counter += 1

        # 深度图像处理
        if img_depth_conversion:
            img_depth *= 16
        if depth != 0:  # 若已识别且成功获取距离
            pt1 = (int(bu1), int(bv1))
            pt2 = (int(bu2), int(bv2))
            cv2.rectangle(img_depth, pt1=pt1, pt2=pt2, color=0xffff)
        self.q_depth.put(img_depth)  # 向主线程发送深度图像。


def main():
    rclpy.init()
    node = ObjectDetectionService()

    # 在另一个线程中运行 rclpy.spin()
    executor = MultiThreadedExecutor()
    thread = threading.Thread(
        target=rclpy.spin, args=(node, executor), daemon=True)
    thread.start()

    try:
        # 由于 imshow() 规定必须在主线程中调用，因此在此执行。
        while True:
            if not node.q_color.empty():
                cv2.imshow('color', node.q_color.get())
            if not node.q_depth.empty():
                cv2.imshow('depth', node.q_depth.get())
            cv2.waitKey(1)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()