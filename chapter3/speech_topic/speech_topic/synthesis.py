import rclpy #[*] 导入用于在 Python 中使用 ROS2 的模块。
import rclpy.node
from std_msgs.msg import String

from gtts import gTTS #[*] gTTS 是一个可以使用 Google 语音合成 API 的模块。
from io import BytesIO #[*] 用于播放获取的语音合成数据的模块。
from mpg123 import Mpg123, Out123


class Synthesis(rclpy.node.Node): #[*] 为了将 Synthesis 类作为节点使用，继承 Node 类。
    def __init__(self):
        super().__init__('speech_synthesis') #[*] 将节点名称注册为 'speech_synthesis'。

        self.get_logger().info('启动语音合成节点') #[*] 使用 logger 显示节点已启动。

        self.lang = 'zh' #[*] 设置希望进行语音合成的语言。
        self.mp3 = Mpg123() #[*] 初始化用于播放语音合成数据的类。
        self.out = Out123()

        self.subscriber = self.create_subscription( #[*] 初始化用于接收待合成文本的 Subscription。接收到的文本由 synthesis 函数处理。
            String, '/speech', self.synthesis, 1)

    def synthesis(self, msg):
        if msg.data != '': #[*] 当获取到需要语音合成的文本时
            self.get_logger().info('正在执行语音合成')

            text = msg.data
            self.get_logger().info(f'将朗读：“{text}”')

            tts = gTTS(text, lang=self.lang[:2]) #[*] 将需要语音合成的文本传递给 Google 语音合成 API。
            fp = BytesIO() #[*] 初始化用于播放语音合成数据的 IO。
            tts.write_to_fp(fp)
            fp.seek(0)
            self.mp3.feed(fp.read())

            for frame in self.mp3.iter_frames(self.out.start): #[*] 播放语音合成数据。
                self.out.play(frame)


def main():
    rclpy.init() #[*] 使能通过 rclpy 进行 ROS 通信。

    synthesis_node = Synthesis() #[*] 初始化语音合成节点。

    try:
        rclpy.spin(synthesis_node) #[*] 运行语音合成节点。
    except KeyboardInterrupt:
        pass

    rclpy.shutdown() #[*] 关闭通过 rclpy 进行的 ROS 通信。