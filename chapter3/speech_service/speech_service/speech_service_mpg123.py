import rclpy
import rclpy.node
from airobot_interfaces.srv import StringCommand

from gtts import gTTS
import speech_recognition as sr
import subprocess


class SpeechService(rclpy.node.Node):
    def __init__(self):
        super().__init__('speech_service')

        self.get_logger().info('语音服务器已启动')

        self.init_rec = sr.Recognizer()

        self.service = self.create_service(
            StringCommand, '/speech_service/wake_up', self.command_callback)

    def command_callback(self, request, response):

        self.synthesis('I\'m ready.')

        text = None
        while text is None:
            text = self.recognition()

        self.synthesis(text)

        response.answer = text
        return response

    def recognition(self):
        text = ''

        with sr.Microphone() as source:
            while text == '':
                audio_data = self.init_rec.record(source, duration=5)
                self.get_logger().info('正在进行语音识别')

                try:
                    text = self.init_rec.recognize_whisper(audio_data, model="medium", language="chinese") #[*] 将录音数据发送给 Whisper，并接收语音识别结果。
                    # text = self.init_rec.recognize_google(audio_data)
                except sr.UnknownValueError:
                    pass

        self.get_logger().info(f'识别出的文本为 "{text}"')

        return text

    def synthesis(self, text):

        self.get_logger().info('正在执行语音合成')
        self.get_logger().info(f'要朗读的内容是 "{text}"')

        gTTS(text, lang='en').save('voice.mp3')

        subprocess.run(['mpg123 voice.mp3'], shell=True)


def main():
    rclpy.init()

    speech_service = SpeechService()

    try:
        rclpy.spin(speech_service)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()