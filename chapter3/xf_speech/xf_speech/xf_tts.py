#!/usr/bin/env python
# -*- coding:utf-8 -*-
import websocket
import datetime
import hashlib
import base64
import hmac
import json
from urllib.parse import urlencode
import time
import ssl
from wsgiref.handlers import format_date_time
from datetime import datetime
from time import mktime
import _thread as thread
import os
import sys
import threading
import pyaudio
from pathlib import Path

# --- 保留你原有的 config 导入方式 ---
pkg_path = str(Path(__file__).resolve().parents[1])
sys.path.insert(0, pkg_path + "/config")
import xf_config

STATUS_FIRST_FRAME = 0
STATUS_CONTINUE_FRAME = 1
STATUS_LAST_FRAME = 2


class Ws_Param(object):
    def __init__(self, APPID, APIKey, APISecret, Text):
        self.APPID = APPID
        self.APIKey = APIKey
        self.APISecret = APISecret
        self.Text = Text
        self.CommonArgs = {"app_id": self.APPID}
        self.BusinessArgs = {"aue": "raw", "auf": "audio/L16;rate=16000", "vcn": "xiaoyan", "tte": "utf8"}
        self.Data = {"status": STATUS_FIRST_FRAME, "text": str(base64.b64encode(self.Text.encode('utf-8')), "UTF8")}

    def create_url(self):
        url = 'wss://tts-api.xfyun.cn/v2/tts'
        now = datetime.now()
        date = format_date_time(mktime(now.timetuple()))
        signature_origin = "host: ws-api.xfyun.cn\ndate: {}\nGET /v2/tts HTTP/1.1".format(date)
        signature_sha = hmac.new(
            self.APISecret.encode('utf-8'),
            signature_origin.encode('utf-8'),
            digestmod=hashlib.sha256
        ).digest()
        signature_sha = base64.b64encode(signature_sha).decode('utf-8')
        authorization_origin = 'api_key="{}", algorithm="hmac-sha256", headers="host date request-line", signature="{}"'.format(
            self.APIKey, signature_sha
        )
        authorization = base64.b64encode(authorization_origin.encode('utf-8')).decode('utf-8')
        v = {
            "authorization": authorization,
            "date": date,
            "host": "ws-api.xfyun.cn"
        }
        return url + '?' + urlencode(v)


# ===== 新增：全局播放控制 =====
class TTSPlayer:
    def __init__(self, stop_event=None):
        self.stop_event = stop_event or threading.Event()
        self.audio_buffer = []
        self.buffer_lock = threading.Lock()
        self.finished = threading.Event()
        self.p = pyaudio.PyAudio()
        self.stream = None
        self.error = None

    def start_playback(self):
        """启动播放线程"""
        def play():
            try:
                self.stream = self.p.open(
                    format=pyaudio.paInt16,
                    channels=1,
                    rate=16000,
                    output=True,
                    frames_per_buffer=1024
                )
                while not self.stop_event.is_set():
                    with self.buffer_lock:
                        if self.audio_buffer:
                            chunk = self.audio_buffer.pop(0)
                            self.stream.write(chunk)
                        else:
                            if self.finished.is_set() and not self.audio_buffer:
                                break
                            time.sleep(0.01)
                    if self.stop_event.is_set():
                        break
            except Exception as e:
                self.error = str(e)
            finally:
                if self.stream:
                    self.stream.stop_stream()
                    self.stream.close()
                self.p.terminate()

        threading.Thread(target=play, daemon=True).start()

    def feed_audio(self, audio_data):
        with self.buffer_lock:
            self.audio_buffer.append(audio_data)

    def finish(self):
        self.finished.set()

    def is_error(self):
        return self.error is not None

    def get_error(self):
        return self.error


# ===== 修改 on_message：不再写文件，而是喂给播放器 =====
def make_on_message(player):
    def on_message(ws, message):
        try:
            msg = json.loads(message)
            code = msg["code"]
            if code != 0:
                print("讯飞TTS错误:", msg.get("message", ""), "code:", code)
                ws.close()
                return

            audio_b64 = msg["data"]["audio"]
            audio = base64.b64decode(audio_b64)
            player.feed_audio(audio)

            status = msg["data"]["status"]
            if status == STATUS_LAST_FRAME:
                player.finish()
                ws.close()
        except Exception as e:
            print("解析音频帧失败:", e)
            ws.close()
    return on_message


def tts_fun(string_txt, stop_event=None):
    # 初始化播放器
    player = TTSPlayer(stop_event=stop_event)
    player.start_playback()

    def on_open(ws):
        def run():
            wsParam = Ws_Param(
                APPID=xf_config.APPID,
                APIKey=xf_config.API_KEY,
                APISecret=xf_config.API_SECRET,
                Text=string_txt
            )
            d = {
                "common": wsParam.CommonArgs,
                "business": wsParam.BusinessArgs,
                "data": wsParam.Data,
            }
            ws.send(json.dumps(d))
        thread.start_new_thread(run, ())

    # 创建 WebSocket
    wsParam = Ws_Param(xf_config.APPID, xf_config.API_KEY, xf_config.API_SECRET, string_txt)
    websocket.enableTrace(False)
    wsUrl = wsParam.create_url()

    ws = websocket.WebSocketApp(
        wsUrl,
        on_message=make_on_message(player),
        on_error=lambda ws, err: print("WebSocket error:", err),
        on_close=lambda ws, a, b: None
    )
    ws.on_open = on_open

    # 启动连接（会阻塞直到合成结束或出错）
    ws.run_forever(sslopt={"cert_reqs": ssl.CERT_NONE})

    # 等待播放完成（最多等3秒，避免卡死）
    if not player.is_error():
        for _ in range(30):  # 最多等3秒
            if player.finished.is_set() and not player.audio_buffer:
                break
            if stop_event and stop_event.is_set():
                break
            time.sleep(0.1)

    if player.is_error():
        raise RuntimeError("播放出错: " + player.get_error())


def tts_main(input_txt, stop_event=None):
    """
    主入口函数，支持传入 stop_event 实现中断
    :param input_txt: 要合成的文本
    :param stop_event: threading.Event，设为 set() 可中断播放
    """
    tts_fun(input_txt, stop_event=stop_event)


# 兼容旧接口（可选）
class tts_clss:
    def __init__(self, input_txt, stop_event=None):
        tts_main(input_txt, stop_event)


if __name__ == "__main__":
    # 示例：5秒后自动中断
    stop_evt = threading.Event()
    t = threading.Thread(target=tts_main, args=("你好，这是一段很长的测试语音，你可以试试在中途按 Ctrl+C 中断它。", stop_evt))
    t.start()
    try:
        time.sleep(5)
        print(">>> 发送中断信号 <<<")
        stop_evt.set()
    except KeyboardInterrupt:
        stop_evt.set()
    t.join()