import rclpy
import rclpy.node
from airobot_interfaces.srv import StringCommand

from gtts import gTTS
import speech_recognition as sr
from io import BytesIO
from mpg123 import Mpg123, Out123


class SpeechService(rclpy.node.Node):
    def __init__(self):
        super().__init__('speech_service')

        self.get_logger().info('语音服务已启动')

        # 初始化语音识别器
        self.init_rec = sr.Recognizer()

        # 创建一个名为 '/speech_service/wake_up' 的服务，类型为 StringCommand
        self.service = self.create_service(
            StringCommand, '/speech_service/wake_up', self.command_callback)

        # 设置默认语言（例如 'en' 表示英语）
        self.lang = 'en'
        # 初始化 MP3 解码器和音频输出设备
        self.mp3 = Mpg123()
        self.out = Out123()

    def command_callback(self, request, response):
        """
        服务回调函数：
        - 先播报 "I'm ready."
        - 然后持续监听麦克风，直到识别出有效语音
        - 将识别结果通过语音复述一遍，并作为服务响应返回
        """

        self.synthesis("I'm ready.")

        text = ''
        # 循环直到成功识别出非空文本
        while text == '':
            text = self.recognition()

        # 将识别到的文本用语音朗读出来
        self.synthesis(text)

        # 将识别结果写入响应
        response.answer = text
        return response

    def recognition(self):
        """使用 Google Web Speech API 进行语音识别"""

        text = ''

        with sr.Microphone() as source:
            # 持续录音并尝试识别，直到获得有效结果
            while text == '':
                # 录制 5 秒音频
                audio_data = self.init_rec.record(source, duration=5)
                self.get_logger().info('正在执行语音识别...')

                try:
                    # 使用 Google 服务进行识别（需联网）
                    text = self.init_rec.recognize_google(audio_data)
                except sr.UnknownValueError:
                    # 若无法识别语音，继续下一轮录音
                    pass

        self.get_logger().info(f'识别到的文本为："{text}"')
        return text

    def synthesis(self, text):
        """使用 gTTS（Google Text-to-Speech）将文本合成为语音并播放"""

        self.get_logger().info('正在执行语音合成...')
        self.get_logger().info(f'即将播报的内容："{text}"')

        # 使用指定语言生成 TTS 音频（取前两个字符如 'en'）
        tts = gTTS(text, lang=self.lang[:2])
        fp = BytesIO()
        tts.write_to_fp(fp)
        fp.seek(0)

        # 将生成的 MP3 数据喂给解码器
        self.mp3.feed(fp.read())

        # 初始化音频输出并逐帧播放
        for frame in self.mp3.iter_frames(self.out.start):
            self.out.play(frame)


def main():
    rclpy.init()

    speech_service = SpeechService()

    try:
        rclpy.spin(speech_service)  # 保持节点运行，监听服务请求
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()