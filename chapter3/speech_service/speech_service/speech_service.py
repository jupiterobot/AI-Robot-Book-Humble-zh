import rclpy #[*] 导入用于在 Python 中使用 ROS2 的模块。
import rclpy.node
from airobot_interfaces.srv import StringCommand

from gtts import gTTS #[*] gTTS 是一个可以使用 Google 语音合成 API 的模块。
import speech_recognition as sr #[*] 导入用于语音识别的模块。
from io import BytesIO #[*] 用于播放获取的语音合成数据的模块。
from mpg123 import Mpg123, Out123


class SpeechService(rclpy.node.Node): #[*] 为了将 SpeechService 类作为节点使用，继承 Node 类。
    def __init__(self):
        super().__init__('speech_service') #[*] 将节点名称注册为 'speech_service'。
     
        self.get_logger().info('语音服务器已启动') #[*] 使用 logger 显示节点已启动。
        
        self.init_rec = sr.Recognizer() #[*] 初始化语音识别器。

        self.service = self.create_service( #[*] 创建一个用于接收启动复读服务命令的 Service。收到请求时，调用 command_callback 函数。
            StringCommand, '/speech_service/wake_up', self.command_callback)

        self.lang = 'en' #[*] 设置希望进行语音合成的语言。
        self.mp3 = Mpg123() #[*] 初始化用于播放语音合成数据的类。
        self.out = Out123()
        
    def command_callback(self, request, response):

        self.synthesis("I'm ready.") #[*] 通过语音合成告知服务已启动并发声。
        
        text = '' #[*] 持续进行语音识别，直到获得识别出的文本。
        while text == '':
            text = self.recognition()

        self.synthesis(text) #[*] 对识别出的文本进行语音合成。

        response.answer = text #[*] 将识别结果作为 Service 的响应返回。
        return response

    def recognition(self):
        text = ''

        with sr.Microphone() as source: #[*] 使录音数据可被处理。
            while text == '':
                audio_data = self.init_rec.record(source, duration=5) #[*] 获取 5 秒钟的录音数据。
                self.get_logger().info(f'正在进行语音识别')

                try:
                    text = self.init_rec.recognize_whisper(audio_data, model="medium", language="chinese") #[*] 将录音数据发送给 Whisper，并接收语音识别结果。
                    # text = self.init_rec.recognize_google(audio_data) #[*] 将录音数据发送给 Google 语音识别器，并接收语音识别结果。
                except sr.UnknownValueError:
                    pass

        self.get_logger().info(f'识别出的文本为 "{text}"') #[*] 使用 logger 显示识别结果。

        return text

    def synthesis(self, text):

        self.get_logger().info(f'正在执行语音合成') #[*] 使用 logger 显示正在执行语音合成。
        self.get_logger().info(f'要朗读的内容是 "{text}"')

        tts = gTTS(text, lang=self.lang[:2]) #[*] 将需要语音合成的文本传递给 Google 语音合成 API。
        fp = BytesIO() #[*] 初始化用于播放语音合成数据的 IO。
        tts.write_to_fp(fp)
        fp.seek(0)
        self.mp3.feed(fp.read())

        for frame in self.mp3.iter_frames(self.out.start): #[*] 播放语音合成数据。
            self.out.play(frame)


def main():
    rclpy.init() #[*] 使能通过 rclpy 进行 ROS 通信。

    speech_service = SpeechService() #[*] 初始化复读服务。

    try:
        rclpy.spin(speech_service) #[*] 运行语音识别节点。
    except KeyboardInterrupt:
        pass
    
    rclpy.shutdown() #[*] 关闭通过 rclpy 进行的 ROS 通信。