# speech_service
## 概要
第3章的示例程序  
执行语音识别与语音合成的程序


## 执行
  
- 基于服务的复读功能执行步骤
  - 启动语音服务：
    ```
    ros2 run speech_service speech_service
    ```
  - 向语音服务发送启动命令：
    ```
    ros2 service call /speech_service/wake_up airobot_interfaces/srv/StringCommand "{command: 'start'}"
    ```

## 帮助
- 本示例程序在 Docker 容器中运行时，仅确认在 Ubuntu 作为主机的情况下可正常工作。使用 Windows 开发的用户，可通过 VMware 等虚拟机安装 Ubuntu，并在其中运行本示例程序。
- 在语音识别过程中，若要更改 Whisper 的模型大小或识别语言，请修改 speech_service.py 中 recognize_whisper 函数的参数。model 可选值请参考 [https://github.com/openai/whisper#available-models-and-languages](https://github.com/openai/whisper#available-models-and-languages)，language 可选值请参考 [https://github.com/openai/whisper/blob/main/whisper/tokenizer.py](https://github.com/openai/whisper/blob/main/whisper/tokenizer.py)，并填写支持的模型和语言。
- 在基于服务的复读功能执行过程中，如果声音未从扬声器输出，请尝试运行 speech_service_mpg123.py。该脚本会输出语音合成后的 mp3 文件，可播放该文件进行确认。

## 作者
萩原　良信

## 历史
- 2022-08-28: 初版
- 2024-01-26: 修订版

## 许可证
Copyright (c) 2022-2025, Yoshinobu Hagiwara, Okuma Yuki, Valentin Cardenas Keith, Masaki Ito and Shoichi Hasegawa
All rights reserved.
本项目采用 Apache-2.0 许可证，许可证全文见本项目根目录下的 LICENSE 文件。

## 参考文献