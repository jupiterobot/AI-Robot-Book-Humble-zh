#! /usr/bin/env python
# -*- coding:utf-8 -*-
import wave
import pyaudio
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
from pathlib import Path
import sys
import threading

STATUS_FIRST_FRAME = 0  # 第一帧的标识
STATUS_CONTINUE_FRAME = 1  # 中间帧标识
STATUS_LAST_FRAME = 2  # 最后一帧的标识

RESULT = ""  # 返回结果的全局变量

pkg_path = str(Path(__file__).resolve().parents[1])  # 获取pkg路径
sys.path.insert(0, pkg_path + "/config")  # 获取pkg/config路径
import xf_config


class Ws_Param(object):
    def __init__(self, APPID, APIKey, APISecret, AudioFile):
        self.APPID = APPID
        self.APIKey = APIKey
        self.APISecret = APISecret
        self.AudioFile = AudioFile
        self.CommonArgs = {"app_id": self.APPID}
        self.BusinessArgs = {"domain": "iat", "language": "zh_cn", "accent": "mandarin", "vinfo": 1, "vad_eos": 10000}

    def create_url(self):
        url = 'wss://ws-api.xfyun.cn/v2/iat'
        now = datetime.now()
        date = format_date_time(mktime(now.timetuple()))

        signature_origin = "host: " + "ws-api.xfyun.cn" + "\n"
        signature_origin += "date: " + date + "\n"
        signature_origin += "GET " + "/v2/iat " + "HTTP/1.1"
        signature_sha = hmac.new(
            self.APISecret.encode('utf-8'),
            signature_origin.encode('utf-8'),
            digestmod=hashlib.sha256
        ).digest()
        signature_sha = base64.b64encode(signature_sha).decode('utf-8')

        authorization_origin = 'api_key="%s", algorithm="%s", headers="%s", signature="%s"' % (
            self.APIKey, "hmac-sha256", "host date request-line", signature_sha
        )
        authorization = base64.b64encode(authorization_origin.encode('utf-8')).decode('utf-8')
        v = {
            "authorization": authorization,
            "date": date,
            "host": "ws-api.xfyun.cn"
        }
        return url + '?' + urlencode(v)


def on_message(ws, message):
    global RESULT
    try:
        data_json = json.loads(message)
        code = data_json["code"]
        sid = data_json["sid"]
        if code != 0:
            errMsg = data_json["message"]
            print("sid:%s call error:%s code is:%s" % (sid, errMsg, code))
        else:
            data = data_json["data"]["result"]["ws"]
            result = ""
            for i in data:
                for w in i["cw"]:
                    result += w["w"]
            RESULT += result.replace('\n', '').replace('\r', '')
    except Exception as e:
        print("receive msg, but parse exception:", e)


def on_error(ws, error):
    pass


def on_close(ws):
    print("### closed ###")


def record(time_sec, stop_event=None):
    """
    录音函数，支持通过 stop_event 中断。
    """
    CHUNK = 1024
    FORMAT = pyaudio.paInt16
    CHANNELS = 1
    RATE = 16000
    WAVE_OUTPUT_FILENAME = "./output.pcm"

    p = pyaudio.PyAudio()

    try:
        stream = p.open(
            format=FORMAT,
            channels=CHANNELS,
            rate=RATE,
            input=True,
            frames_per_buffer=CHUNK
        )
    except OSError as e:
        print(f"[ERROR] Failed to open audio stream: {e}")
        p.terminate()
        return False

    print("* recording")
    frames = []
    total_chunks = int(RATE / CHUNK * time_sec)

    for i in range(total_chunks):
        if stop_event and stop_event.is_set():
            print("* recording interrupted by stop event")
            break
        try:
            data = stream.read(CHUNK, exception_on_overflow=False)
            frames.append(data)
        except Exception as e:
            print(f"[WARN] Read audio chunk failed: {e}")
            break

    print("* done recording")
    stream.stop_stream()
    stream.close()
    p.terminate()

    # 即使被中断，也保存已录制的部分
    wf = wave.open(WAVE_OUTPUT_FILENAME, 'wb')
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(p.get_sample_size(FORMAT))
    wf.setframerate(RATE)
    wf.writeframes(b''.join(frames))
    wf.close()
    return True


def iat_fun(input_t, stop_event=None):
    global RESULT
    RESULT = ""

    # 先录音（支持中断）
    success = record(input_t, stop_event=stop_event)
    if not success:
        return ""

    # 如果录音刚结束就被取消，可提前返回
    if stop_event and stop_event.is_set():
        return ""

    wsParam = Ws_Param(
        APPID=xf_config.APPID,
        APIKey=xf_config.API_KEY,
        APISecret=xf_config.API_SECRET,
        AudioFile=r'./output.pcm'
    )

    def on_open(ws):
        def run():
            frameSize = 8000
            intervel = 0.04
            status = STATUS_FIRST_FRAME

            with open(wsParam.AudioFile, "rb") as fp:
                while True:
                    # 检查是否被要求中断
                    if stop_event and stop_event.is_set():
                        print("* recognition interrupted during sending")
                        ws.close()
                        return

                    buf = fp.read(frameSize)
                    if not buf:
                        status = STATUS_LAST_FRAME

                    d = {
                        "data": {
                            "status": status,
                            "format": "audio/L16;rate=16000",
                            "audio": str(base64.b64encode(buf), 'utf-8'),
                            "encoding": "raw"
                        }
                    }

                    if status == STATUS_FIRST_FRAME:
                        d["common"] = wsParam.CommonArgs
                        d["business"] = wsParam.BusinessArgs
                        status = STATUS_CONTINUE_FRAME

                    ws.send(json.dumps(d))

                    if status == STATUS_LAST_FRAME:
                        time.sleep(1)
                        break

                    time.sleep(intervel)

            ws.close()

        thread.start_new_thread(run, ())

    websocket.enableTrace(False)
    wsUrl = wsParam.create_url()
    ws = websocket.WebSocketApp(wsUrl, on_message=on_message, on_error=on_error, on_close=on_close)
    ws.on_open = on_open
    ws.run_forever(sslopt={"cert_reqs": ssl.CERT_NONE})

    return RESULT


def iat_main(input_t, stop_event=None):
    """
    主入口函数，支持传入 stop_event 实现中断。
    :param input_t: 录音时长（秒）
    :param stop_event: threading.Event 对象，用于外部中断
    :return: 识别结果字符串
    """
    output = iat_fun(input_t, stop_event=stop_event)
    return output


if __name__ == "__main__":
    # 示例：10秒录音，不可中断（因为没传 stop_event）
    r = iat_main(input_t=10)
    print("Final result:", r)