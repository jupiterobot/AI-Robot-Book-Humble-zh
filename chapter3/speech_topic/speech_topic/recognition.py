import rclpy #[*] 导入用于在 Python 中使用 ROS2 的模块。
import rclpy.node
from std_msgs.msg import String

import speech_recognition as sr #[*] 导入用于语音识别的模块。

class Recognition(rclpy.node.Node): #[*] 为了将 Recognition 类作为节点使用，继承 Node 类。
    def __init__(self):
        super().__init__('speech_recognition') #[*] 将节点名称注册为 'speech_recognition'。

        self.get_logger().info('启动语音识别节点') #[*] 使用 logger 显示节点已启动。

        self.init_rec = sr.Recognizer() #[*] 初始化语音识别器。

        self.publisher = self.create_publisher(String, '/speech', 1) #[*] 为了将识别出的语音发送到 /speech 主题，初始化 Publisher。

        self.timer = self.create_timer(5.0, self.recognition) #[*] 设置每 5 秒执行一次语音识别，因此初始化 Timer。要执行的函数是 recognition。

    def recognition(self):
        with sr.Microphone() as source: #[*] 使录音数据可被处理。
            text = ''

            audio_data = self.init_rec.record(source, duration=5) #[*] 获取 5 秒钟的录音数据。
                
            self.get_logger().info(f'正在进行语音识别')

            try:
                text = self.init_rec.recognize_whisper(audio_data, model="medium", language="chinese") #[*] 将录音数据发送给 Whisper，并接收语音识别结果。
                # text = self.init_rec.recognize_google(audio_data) #[*] 将录音数据发送给 Google 语音识别器，并接收语音识别结果。

            except sr.UnknownValueError:
                pass

            if text != '': #[*] 如果获得了语音识别结果，则将结果发布到 /speech 主题。
                msg = String()
                msg.data = text
                self.get_logger().info(
                    f'将识别出的语音 "{text}" 发布到主题 /speech')

                self.publisher.publish(msg)

def main():
    rclpy.init() #[*] 使能通过 rclpy 进行 ROS 通信。

    recognition_node = Recognition() #[*] 初始化语音识别节点。
    
    try:
        rclpy.spin(recognition_node) #[*] 运行语音识别节点。
    except KeyboardInterrupt:
        pass

    rclpy.shutdown() #[*] 关闭通过 rclpy 进行的 ROS 通信。