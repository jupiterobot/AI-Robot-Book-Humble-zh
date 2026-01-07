import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.action.server import ServerGoalHandle
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

from airobot_interfaces.action import StringCommand


class ObjectDetectionActionServer(Node):

    def __init__(self):
        super().__init__('object_detection_action_server')
        self.get_logger().info('图像识别服务器启动中')

        self.running = False  # 物体检测处理标志
        self.target_name = 'cup'  # 要查找的物体名称
        self.frame_id = 'target'  # 广播的 tf 名称
        self.counter_total = 0  # 接收图像计数器
        self.counter_detect = 0  # 检测到物体的计数器
        self.q_color = queue.Queue()  # 向主线程发送彩色图像的队列
        self.q_depth = queue.Queue()  # 向主线程发送深度图像的队列
        self.goal_handle = None
        self.goal_lock = threading.Lock()
        self.execute_lock = threading.Lock()
        self.target_detection_lock = threading.Lock()

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

        # 接收来自其他节点指令的动作服务器
        self.action_server = ActionServer(
            self,
            StringCommand,
            'vision/command',
            self.execute_callback,
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            callback_group=self.callback_group
        )

        self.model = YOLO('yolov8m.pt')
        # 通过执行一次虚拟图像推理来加载 YOLO 缓存
        dummy_image = np.zeros((640, 480, 3), dtype=np.uint8)
        results = self.model(dummy_image, verbose=False)
        self.names = results[0].names.values()
        self.get_logger().info('图像识别服务器已启动完成')
        threading.excepthook = lambda x: ()

    def handle_accepted_callback(self, goal_handle):
        with self.goal_lock:
            if self.goal_handle is not None and self.goal_handle.is_active:
                self.get_logger().info('正在中止前一个目标')
                self.goal_handle.abort()
            self.goal_handle = goal_handle
        goal_handle.execute()

    def execute_callback(self, goal_handle: ServerGoalHandle):
        with self.execute_lock:
            self.get_logger().info('执行中...')
            request: StringCommand.Goal = goal_handle.request
            result = StringCommand.Result()
            result.answer = 'NG'
            # 指令 find: 在限定时间内尝试检测物体
            if request.command.startswith('find'):
                name = request.command[4:].strip()
                if len(name) == 0:
                    result.answer = 'NG name required'
                    goal_handle.abort()
                elif name not in self.names:
                    result.answer = 'NG unknown name'
                    goal_handle.abort()
                else:
                    with self.target_detection_lock:  # 开始物体检测
                        self.target_name = name
                        self.running = True
                        self.counter_total = 0
                        self.counter_detect = 0
                    start_time = time.time()
                    while time.time() - start_time < 3:
                        if not goal_handle.is_active:
                            with self.target_detection_lock:
                                self.running = False
                            break
                        if goal_handle.is_cancel_requested:
                            goal_handle.canceled()
                            with self.target_detection_lock:
                                self.running = False
                            break
                        time.sleep(0.1)
                    if self.running:
                        with self.target_detection_lock:
                            counter_total = self.counter_total
                            counter_detect = self.counter_detect
                            self.running = False
                        if counter_total > 0 and counter_detect / counter_total >= 0.5:
                            result.answer = 'OK'
                            goal_handle.succeed()
                        else:
                            result.answer = 'NG not found'
                            goal_handle.succeed()
            # 指令 track: 持续进行物体检测
            elif request.command.startswith('track'):
                name = request.command[5:].strip()
                if len(name) == 0:
                    result.answer = 'NG name required'
                    goal_handle.abort()
                elif name not in self.names:
                    result.answer = 'NG unknown name'
                    goal_handle.abort()
                else:
                    with self.target_detection_lock:  # 开始物体检测
                        self.target_name = name
                        self.running = True
                    result.answer = 'OK'
                    goal_handle.succeed()
            # 指令 stop: 停止物体检测处理
            elif request.command.startswith('stop'):
                with self.target_detection_lock:
                    self.running = False
                result.answer = 'OK'
                goal_handle.succeed()
            # 其他指令
            else:
                result.answer = f'NG {request.command} not supported'
                goal_handle.abort()

            self.get_logger().info(f'answer: {result.answer}')
            return result

    def cancel_callback(self, goal_handle):
        self.get_logger().info('收到取消请求')
        return CancelResponse.ACCEPT

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

        # 与动作执行回调方法共享
        with self.target_detection_lock:
            target_name = self.target_name
            running = self.running

        # 物体检测
        boxes = []
        classes = []
        if running:
            results = self.model(img_color, verbose=False)
            names = results[0].names
            boxes = results[0].boxes
            classes = results[0].boxes.cls
            img_color = results[0].plot()
            with self.target_detection_lock:
                self.counter_total += 1

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
                # 与动作执行回调方法共享
                with self.target_detection_lock:
                    self.counter_detect += 1

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
    node = ObjectDetectionActionServer()
    executor = MultiThreadedExecutor()
    thread = threading.Thread(target=rclpy.spin, args=(node, executor), daemon=True)
    thread.start()

    try:
        # 由于 imshow() 规定必须在主线程中调用，因此在此执行。
        while True:
            if not node.q_color.empty():
                cv2.imshow('object_detection color', node.q_color.get())
            if not node.q_depth.empty():
                cv2.imshow('object_detection depth', node.q_depth.get())
            cv2.waitKey(1)
    except KeyboardInterrupt:
        pass

    rclpy.try_shutdown()