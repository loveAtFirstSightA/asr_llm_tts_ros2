#!/home/lio/miniconda3/envs/asr_llm_tts/bin/python
# -*- coding:utf-8 -*-

"""
 Copyright 2025 Author lio

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

      https://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
 """

import sys
# TODO 加载conda环境
sys.path.append(f'/home/lio/miniconda3/envs/asr_llm_tts/lib/python3.10/site-packages')
import os
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int64
import requests
import pygame
import websocket
import datetime
import hashlib
import base64
import hmac
import json
from urllib.parse import urlencode
import ssl
from wsgiref.handlers import format_date_time
from time import mktime
from loguru import logger
import torch
from TTS.api import TTS as coqui_tts
from TTS.tts.configs.xtts_config import XttsConfig
from TTS.tts.models.xtts import XttsAudioConfig, XttsArgs  # 导入 XttsArgs
from TTS.config.shared_configs import BaseDatasetConfig

# Configure loguru output (similar to your previous example)
logger.remove()
logger.add(
    sys.stdout,
    format="[{time:YYYY-MM-DD HH:mm:ss.SSS}] [{level}] {message}",
    level="INFO"
)
logger.info('Using loguru log system')

# Constants for frame statuses
STATUS_FIRST_FRAME = 0      # First frame indicator
STATUS_CONTINUE_FRAME = 1   # Intermediate frame indicator
STATUS_LAST_FRAME = 2       # Last frame indicator

class WsParam:
    """
    Encapsulates parameters for ifly TTS WebSocket requests.
    """
    def __init__(self, app_id, api_key, api_secret, text):
        self.app_id = app_id
        self.api_key = api_key
        self.api_secret = api_secret
        self.text = text

        # Public parameters
        self.common_args = {"app_id": self.app_id}
        # Business parameters (additional customization available on the official website)
        self.business_args = {
            "aue": "lame",
            "sfl": 1,
            "auf": "audio/L16;rate=16000",
            "vcn": "xiaoyan",
            "tte": "utf8"
        }
        # Text data (Base64 encoded)
        self.data = {
            "status": STATUS_LAST_FRAME,
            "text": base64.b64encode(self.text.encode('utf-8')).decode("utf-8")
        }

    def create_url(self):
        """
        Generate the WebSocket URL for connection.
        """
        base_url = 'wss://tts-api.xfyun.cn/v2/tts'
        now = datetime.datetime.now()
        date = format_date_time(mktime(now.timetuple()))
        # Construct the signature string
        signature_origin = f"host: ws-api.xfyun.cn\n" \
                           f"date: {date}\n" \
                           f"GET /v2/tts HTTP/1.1"
        # Create HMAC-SHA256 signature
        signature_sha = hmac.new(
            self.api_secret.encode('utf-8'),
            signature_origin.encode('utf-8'),
            digestmod=hashlib.sha256
        ).digest()
        signature_sha_b64 = base64.b64encode(signature_sha).decode('utf-8')
        authorization_origin = (f'api_key="{self.api_key}", algorithm="hmac-sha256", '
                                f'headers="host date request-line", signature="{signature_sha_b64}"')
        authorization = base64.b64encode(authorization_origin.encode('utf-8')).decode('utf-8')
        # Combine authentication parameters
        params = {
            "authorization": authorization,
            "date": date,
            "host": "ws-api.xfyun.cn"
        }
        full_url = f"{base_url}?{urlencode(params)}"
        return full_url

def ifly_text_to_speech(app_id, api_key, api_secret, text, output_file='./demo.wav', logger=logger):
    """
    Convert text to speech using ifly service, and play the resulting audio.
    """
    ws_param = WsParam(app_id, api_key, api_secret, text)
    websocket.enableTrace(False)
    ws_url = ws_param.create_url()

    def on_open(ws):
        def run():
            payload = {
                "common": ws_param.common_args,
                "business": ws_param.business_args,
                "data": ws_param.data
            }
            try:
                ws.send(json.dumps(payload))
                # Remove existing file if present
                if os.path.exists(output_file):
                    os.remove(output_file)
                    logger.info(f"Removed existing file: {output_file}")
            except Exception as e:
                logger.error(f"Error sending data via websocket: {e}")
        threading.Thread(target=run, daemon=True).start()

    def on_message(ws, message):
        try:
            message_json = json.loads(message)
            code = message_json.get("code", -1)
            sid = message_json.get("sid", "")
            audio_b64 = message_json.get("data", {}).get("audio", "")
            audio = base64.b64decode(audio_b64)
            status = message_json.get("data", {}).get("status", 0)
            if status == STATUS_LAST_FRAME:
                ws.close()
            if code != 0:
                errMsg = message_json.get("message", "")
                logger.error(f"sid:{sid} call error: {errMsg} code: {code}")
            else:
                with open(output_file, 'ab') as f:
                    f.write(audio)
        except Exception as e:
            logger.error(f"Error processing websocket message: {e}")

    def on_error(ws, error):
        logger.error(f"WebSocket error: {error}")

    def on_close(ws, close_status_code, close_msg):
        logger.info("WebSocket closed")

    ws_app = websocket.WebSocketApp(
        ws_url,
        on_open=on_open,
        on_message=on_message,
        on_error=on_error,
        on_close=on_close
    )
    ws_app.run_forever(sslopt={"cert_reqs": ssl.CERT_NONE})
    try:
        pygame.mixer.init()
        pygame.mixer.music.load(output_file)
        pygame.mixer.music.play()
    except Exception as e:
        logger.error(f"Error playing sound file {output_file}: {e}")

class TTS(Node):
    def __init__(self):
        super().__init__('tts')
        pygame.mixer.init()
        self.task_interrupt_ = False
        self.input_task_subscriber_ = self.create_subscription(String, 'notice_task', self.input_task_subscriber_callback, 10)
        self.input_file_subscriber_ = self.create_subscription(String, 'file_to_tts', self.input_file_subscriber_callback, 10)
        self.input_text_subscriber_ = self.create_subscription(String, 'text_to_tts', self.input_text_subscriber_callback, 10)
        
        self.declare_parameter('model_type', '1234')
        self.model_type_ = self.get_parameter('model_type').get_parameter_value().string_value
        self.declare_parameter('service_provider', '1234')
        self.service_provider_ = self.get_parameter('service_provider').get_parameter_value().string_value
        self.declare_parameter('baidu_api_key', '1234')
        self.baidu_api_key_ = self.get_parameter('baidu_api_key').get_parameter_value().string_value
        self.declare_parameter('baidu_secret_key', '1234')
        self.baidu_secret_key_ = self.get_parameter('baidu_secret_key').get_parameter_value().string_value
        self.declare_parameter('ifly_appid', '1234')
        self.ifly_appid_ = self.get_parameter('ifly_appid').get_parameter_value().string_value
        self.declare_parameter('ifly_api_secret', '1234')
        self.ifly_api_secret_ = self.get_parameter('ifly_api_secret').get_parameter_value().string_value
        self.declare_parameter('ifly_api_key', '1234')
        self.ifly_api_key_ = self.get_parameter('ifly_api_key').get_parameter_value().string_value
        # TODO debug
        # logger.info('service_provider: %s' % self.service_provider_)
        # logger.info('baidu_api_key %s' % self.baidu_api_key_)
        # logger.info('baidu_secret_key %s' % self.baidu_secret_key_)
        # logger.info('ifly_appid %s' % self.ifly_appid_)
        # logger.info('ifly_api_secret %s' % self.ifly_api_secret_)
        # logger.info('ifly_api_key %s' % self.ifly_api_key_)
        logger.info(f"TTS node initialized with service_provider: {self.service_provider_}")

    def input_task_subscriber_callback(self, msg):
        self.get_logger().info('Received notice_task: %s' % msg.data)
        pygame.mixer.music.stop()
        self.task_interrupt_ = True
        
    def input_file_subscriber_callback(self, msg):
        logger.info(f"Received file message: {msg.data}")
        self.play_audio_async(msg.data)

    def input_text_subscriber_callback(self, msg):
        self.task_interrupt_ = False
        logger.info(f"Received text message: {msg.data}")
        output_file = './output.wav'
        
        # Remove existing audio file if exists
        if os.path.exists(output_file):
            try:
                os.remove(output_file)
                logger.info(f"Removed existing file: {output_file}")
            except Exception as e:
                logger.error(f"Error removing file {output_file}: {e}")
                
        # add offline tts
        if self.model_type_ == "offline":
            logger.info("Using offline TTS method")
            self.coquiai_tts(msg.data)
            
        elif self.model_type_ == "online":
            logger.info("Using online TTS method")
            # Call the appropriate TTS service in a separate thread
            if self.service_provider_.lower() == 'baidu':
                threading.Thread(
                    target=self.baidu_text_to_speech,
                    args=(msg.data, output_file),
                    daemon=True
                ).start()
            elif self.service_provider_.lower() == 'ifly':
                threading.Thread(
                    target=ifly_text_to_speech,
                    args=(self.ifly_appid_, self.ifly_api_key_, self.ifly_api_secret_, msg.data, output_file, logger), # Pass the loguru logger
                    daemon=True
                ).start()
            else:
                logger.error("Invalid service provider specified.")
    
    def coquiai_tts(self, message):
        # 显式允许 XttsConfig, XttsAudioConfig, BaseDatasetConfig 和 XttsArgs 类被加载
        torch.serialization.add_safe_globals([XttsConfig, XttsAudioConfig, BaseDatasetConfig, XttsArgs])
        # Get device 用GPU还是CPU
        device = "cuda" if torch.cuda.is_available() else "cpu"
        # List available TTS models 可以看都有些啥模型名字，注意此时模型文件都没有下载
        #print(TTS().list_models())
        # Init TTS 初始化，传入模型名字，这个路径就得用上面list里的路径，然后下载链接在python安装路径的TTS目录下，这个文件里写的.models.json
        tts = coqui_tts("tts_models/multilingual/multi-dataset/xtts_v2").to(device)

        # Run TTS运行，必须设置语言
        # Text to speech to a file 这是输出到文件了。
        tts.tts_to_file(text=message, speaker_wav="./speaker.wav", language="zh-cn", file_path="output.wav")
        self.play_audio_async("output.wav")

    def baidu_text_to_speech(self, text, output_file):
        logger.info(f"Baidu TTS processing text: {text}")
        url = "https://tsn.baidu.com/text2audio"
        try:
            access_token = self.get_access_token()
            if not access_token:
                logger.error("Failed to obtain access token.")
                return
            params = {
                'tex': text,
                'tok': access_token,
                'cuid': 'ICmQnIbMrcs1nIjVrOIUxFYpoGnQ34WG',
                'ctp': 1,
                'lan': 'zh',
                'spd': 5,
                'pit': 5,
                'vol': 5,
                'per': 4100,
                'aue': 6  # 3 - mp3 6 - wav
            }
            headers = {
                'Content-Type': 'application/x-www-form-urlencoded',
                'Accept': '*/*'
            }
            response = requests.post(url, data=params, headers=headers, timeout=10)
            if response.status_code == 200:
                with open(output_file, "wb") as f:
                    f.write(response.content)
                logger.info(f"Baidu TTS success, audio saved to {output_file}")
                if self.task_interrupt_:
                    self.task_interrupt_ = False
                    logger.warning('interrupt current task')
                    # TODO
                    # self.play_audio_async('/home/lio/asr_llm_tts_ros2/src/tts/resource/repeat.mp3')
                    return
                self.play_audio_async(output_file)
            else:
                logger.error(f"Baidu TTS failed: {response.text}")
        except Exception as e:
            logger.error(f"Exception during Baidu TTS: {e}")

    def get_access_token(self):
        url = "https://aip.baidubce.com/oauth/2.0/token"
        params = {
            "grant_type": "client_credentials",
            "client_id": self.baidu_api_key_,
            "client_secret": self.baidu_secret_key_
        }
        try:
            response = requests.post(url, params=params, timeout=10)
            if response.status_code == 200:
                return response.json().get("access_token", "")
            else:
                logger.error(f"Failed to get access token: {response.text}")
                return None
        except Exception as e:
            logger.error(f"Exception during access token retrieval: {e}")
            return None

    def play_audio_async(self, file_path):
        threading.Thread(target=self.play_audio, args=(file_path,), daemon=True).start()

    def play_audio(self, file_path):
        logger.info('thread - playing %s' % file_path)
        try:
            pygame.mixer.music.stop()
            pygame.mixer.music.load(file_path)
            pygame.mixer.music.play()
        except pygame.error as e:
            logger.error(f"Error playing audio file {file_path}: {e}")
        except Exception as e:
            logger.error(f"An unexpected error occurred during audio playback: {e}")


def main(args=None):
    rclpy.init(args=args)
    tts_node = TTS()
    try:
        rclpy.spin(tts_node)
    except KeyboardInterrupt:
        logger.info("TTS node interrupted by user.")
    finally:
        tts_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()