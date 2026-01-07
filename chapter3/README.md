# 第３章　语音识别・合成（修订第2版）
## 概要
《ROS 2与Python动手学AI机器人入门 修订第2版》（出村・萩原・升谷・坦 著，讲谈社）第３章的示例程序与补充信息等。

## 安装
- 为处理音频相关功能，请使用以下命令安装库：
sudo apt install portaudio19-dev
sudo apt install pulseaudio
- 为在Python中作为模块调用，请执行以下命令：
pip3 install pyaudio
- 使用以下命令安装语音识别库：
pip3 install SpeechRecognition
- 为使用语音识别器Whisper，请使用以下命令安装相关库：
pip3 install SpeechRecognition[whisper-local] soundfile
- 安装语音合成所用的库：
pip3 install gTTS
sudo apt install mpg123
pip3 install mpg123
- 使用以下命令从GitHub克隆示例程序：
cd ~/airobot_ws/src
git clone https://github.com/AI-Robot-Book-Humble/chapter3
- 使用以下命令构建包：
cd ~/airobot_ws
colcon build
source install/setup.bash

## 目录结构
- **[speech_action](speech_action):** 基于动作通信的语音识别与语音合成示例程序
- **[speech_service](speech_service):** 基于服务通信的语音识别与语音合成示例程序
- **[speech_topic](speech_topic):** 基于话题通信的语音识别与语音合成示例程序

## 补充信息
 - 第３章的示例程序在Docker容器中运行时，**仅确认在Ubuntu作为主机的情况下可正常工作**。使用Windows开发的用户，可通过VMWare等虚拟机安装Ubuntu，并在其中运行示例程序。
 - 请事先确认所使用的Ubuntu环境能够从麦克风输入声音并从扬声器输出声音。
 - 若使用虚拟机上的Ubuntu，可能会因延迟导致声音无法输出。请尝试输入较长的语句等方式应对。
 - 如有可能，请使用耳机。在复读功能执行过程中，扬声器的声音可能会被麦克风再次拾取，造成循环重复。