# speech_topic
## 概要
第3章的示例程序  
执行语音识别与语音合成的程序


## 执行
- 语音识别的执行步骤
  - 打开终端，以便能够查看发布到 /speech 话题的数据：
    ```
    ros2 topic echo /speech
    ```
  - 打开新终端，启动语音识别程序：
    ```
    ros2 run speech_topic recognition
    ```
  - 对着麦克风说话。

- 语音合成的执行步骤
  - 打开终端，启动语音合成程序：
    ```
    ros2 run speech_topic synthesis
    ```
  - 打开新终端，向 /speech 话题发送想要播报的消息：
    ```
    ros2 topic pub -1 /speech std_msgs/msg/String "{data: 'I will go to the kitchen and grab a bottle.'}"
    ```
  - 声音将从扬声器输出。

- 基于话题通信的复读功能执行步骤
  - 打开终端，启动语音合成程序：
    ```
    ros2 run speech_topic synthesis
    ```
  - 打开新终端，启动语音识别程序：
    ```
    ros2 run speech_topic recognition
    ```
  - 对着麦克风说话，扬声器会重复你所说的内容。
  

## 帮助
- 本示例程序在 Docker 容器中运行时，仅确认在 Ubuntu 作为主机的情况下可正常工作。使用 Windows 开发的用户，可通过 VMware 等虚拟机安装 Ubuntu，并在其中运行本示例程序。
- 在语音识别的执行过程中，如果声音未从扬声器输出，请尝试运行 synthesis_mpg123.py。该脚本会输出语音合成后的 mp3 文件，可播放该文件进行确认。
- 在基于话题的语音识别执行过程中，若要更改 Whisper 的模型大小或识别语言，请修改 recognition.py 中 recognize_whisper 函数的参数。model 可选值请参考 [https://github.com/openai/whisper#available-models-and-languages](https://github.com/openai/whisper#available-models-and-languages)，language 可选值请参考 [https://github.com/openai/whisper/blob/main/whisper/tokenizer.py](https://github.com/openai/whisper/blob/main/whisper/tokenizer.py)，并填写支持的模型和语言。


## 作者
萩原　良信

## 历史
- 2024-01-26: 修订版
- 2022-08-28: 初版

## 许可证
Copyright (c) 2022-2025, Yoshinobu Hagiwara, Okuma Yuki, Valentin Cardenas Keith, Masaki Ito and Shoichi Hasegawa
All rights reserved.
本项目采用 Apache-2.0 许可证，许可证全文见本项目根目录下的 LICENSE 文件。

## 参考文献