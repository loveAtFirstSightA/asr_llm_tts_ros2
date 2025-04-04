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
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import base64
import json
import requests
import threading
from loguru import logger
from asr import ost_fast
import numpy
import whisper
import wave
import struct

# 配置 loguru 输出到标准输出
logger.remove()
logger.add(
    sys.stdout,
    format="[{time:YYYY-MM-DD HH:mm:ss.SSS}] [{level}] {message}",
    level="INFO"
)
logger.info("Using loguru log system")

class ASR(Node):
    def __init__(self):
        super().__init__('asr')
        self.create_subscription(String, 'pcm_file', self.pcm_subscriber_callback, 10)
        self.asr_result_publisher_ = self.create_publisher(String, 'asr_result', 10)

        # 参数初始化
        self.declare_parameter('function_type', "1234")
        self.function_type_ = self.get_parameter('function_type').get_parameter_value().string_value
        self.declare_parameter('service_provider', "1234")
        self.service_provider_ = self.get_parameter('service_provider').get_parameter_value().string_value
        self.declare_parameter('baidu_api_key', "1234")
        self.baidu_api_key_ = self.get_parameter('baidu_api_key').get_parameter_value().string_value
        self.declare_parameter('baidu_secret_key', "1234")
        self.baidu_secret_key_ = self.get_parameter('baidu_secret_key').get_parameter_value().string_value
        self.declare_parameter('ifly_appid', "1234")
        self.ifly_appid_ = self.get_parameter('ifly_appid').get_parameter_value().string_value
        self.declare_parameter('ifly_apikey', "1234")
        self.ifly_apikey_ = self.get_parameter('ifly_apikey').get_parameter_value().string_value
        self.declare_parameter('ifly_apisecret', "1234")
        self.ifly_apisecret_ = self.get_parameter('ifly_apisecret').get_parameter_value().string_value
        
        logger.info(f"ASR node initialized with service_provider: {self.service_provider_}")
    
    def pcm_subscriber_callback(self, msg: String):
        logger.info(f"Received pcm_file message: {msg.data}")
        # 使用 threading.Thread 来创建子线程，设置为守护线程
        thread = threading.Thread(
            target=self.process_pcm_file, 
            args=(msg.data,), 
            daemon=True
        )
        thread.start()

    def process_pcm_file(self, pcm_file_path: str):
        result = ""
        if self.function_type_ == "online":
            logger.info("调用云端接口进行PCM文件解析")
            try:
                if self.service_provider_ == 'baidu':
                    result = self.baidu_speech_recognition(pcm_file_path)
                elif self.service_provider_ == 'ifly':
                    result = self.ifly_speech_recognition(pcm_file_path)
                else:
                    logger.error(f"未知的 service_provider: {self.service_provider_}")
                    result = None

                if result is None:
                    logger.error("语音识别返回 None 或解析失败")
                    result = "识别失败"
            except Exception as e:
                logger.error(f"处理PCM文件时出现异常: {e}")
                result = "处理异常"
        
        elif self.function_type_ == "offline":
            logger.info("调用离线接口进行PCM文件解析")
            # Convert pcm to wav
            logger.info("Convert pcm to wav")
            pcm_file = pcm_file_path  # 假设 pcm_file_path 已经定义并包含 PCM 文件的路径
            wav_file = pcm_file_path.replace(".pcm", ".wav") # 或者你希望的 WAV 文件名
            sample_rate = 16000
            sample_width = 2
            num_channels = 1 # 如果你的 PCM 是单声道

            self.convert_pcm_to_wav(pcm_file, wav_file, sample_rate, sample_width, num_channels)

            # model type tiny base small medium
            model = whisper.load_model("medium")
            _result = model.transcribe(wav_file)
            logger.info("Openai-whisper result: {}", _result["text"])
            result = _result['text']

        logger.info(f"完成解析: {result}")
        self.publish_result(result)

    def convert_pcm_to_wav(self, pcm_file_path, wav_file_path, sample_rate, sample_width, num_channels):
        """
        将 PCM 文件转换为 WAV 文件。

        Args:
            pcm_file_path (str): PCM 文件的路径。
            wav_file_path (str): 要保存的 WAV 文件的路径。
            sample_rate (int): 采样率 (例如, 1000)。
            sample_width (int): 每个样本的字节数 (例如, 2 代表 16-bit)。
            num_channels (int): 音频通道数 (例如, 1 代表单声道, 2 代表立体声)。
        """
        try:
            with open(pcm_file_path, 'rb') as pcm_file:
                pcm_data = pcm_file.read()

            with wave.open(wav_file_path, 'w') as wav_file:
                wav_file.setnchannels(num_channels)
                wav_file.setsampwidth(sample_width)
                wav_file.setframerate(sample_rate)
                wav_file.writeframes(pcm_data)

            logger.info(f"PCM 文件 '{pcm_file_path}' 已成功转换为 WAV 文件 '{wav_file_path}'。")

        except FileNotFoundError:
            logger.error(f"文件未找到: {pcm_file_path}")
        except Exception as e:
            logger.error(f"转换 PCM 到 WAV 文件时发生错误: {e}")

    def publish_result(self, result: str):
        msg = String()
        msg.data = str(result)
        self.asr_result_publisher_.publish(msg)
        logger.info(f"发布解析结果: {msg.data}")

    def baidu_speech_recognition(self, audio_file_path: str) -> str:
        url = "https://vop.baidu.com/pro_api"
        try:
            with open(audio_file_path, 'rb') as f:
                speech_data = f.read()
        except Exception as e:
            logger.error(f"读取文件失败: {e}")
            return None

        try:
            speech_base64 = base64.b64encode(speech_data).decode('utf-8')
        except Exception as e:
            logger.error(f"base64 编码失败: {e}")
            return None

        try:
            payload = json.dumps({
                "format": "pcm",
                "rate": 16000,
                "channel": 1,
                "cuid": "weePS46GbDUfHkjjlxM0waLes2RP9xSk",
                "dev_pid": 80001,
                "token": self.get_access_token(),
                "speech": speech_base64,
                "len": len(speech_data)
            }, ensure_ascii=False)
        except Exception as e:
            logger.error(f"生成 payload 失败: {e}")
            return None

        headers = {
            'Content-Type': 'application/json',
            'Accept': 'application/json'
        }
        try:
            response = requests.post(url, headers=headers, data=payload.encode("utf-8"), timeout=10)
            response.raise_for_status()
        except requests.exceptions.RequestException as e:
            logger.error(f"HTTP 请求失败: {e}")
            return None

        try:
            response_data = response.json()
            result_list = response_data.get("result", None)
            if result_list and isinstance(result_list, list) and len(result_list) > 0:
                return result_list[0]
            else:
                logger.error(f"没有解析到 result 字段或格式错误: {response_data}")
                return None
        except json.JSONDecodeError as e:
            logger.error(f"JSON 解析失败: {e}")
            return None

    def get_access_token(self) -> str:
        url = "https://aip.baidubce.com/oauth/2.0/token"
        params = {
            "grant_type": "client_credentials",
            "client_id": self.baidu_api_key_,
            "client_secret": self.baidu_secret_key_
        }
        try:
            response = requests.post(url, params=params, timeout=5)
            response.raise_for_status()
            data = response.json()
            token = data.get("access_token", None)
            if token is None:
                logger.error(f"获取 access token 失败: {data}")
            return str(token)
        except requests.exceptions.RequestException as e:
            logger.error(f"获取 access token 请求失败: {e}")
            return None
        except json.JSONDecodeError as e:
            logger.error(f"解析 access token 响应失败: {e}")
            return None

    def ifly_speech_recognition(self, pcm_file: str) -> str:
        logger.info(f"Using iflytec asr service provider processing: {pcm_file}")
        asr_instance = ost_fast.get_result(self.ifly_appid_, self.ifly_apikey_, self.ifly_apisecret_, pcm_file)
        result_info = asr_instance.get_result()

        if isinstance(result_info, dict) and 'data' in result_info and 'result' in result_info['data']:
            asr_result_json = result_info['data']['result']
            if isinstance(asr_result_json, str):
                try:
                    asr_result_json = json.loads(asr_result_json)
                except json.JSONDecodeError:
                    logger.error("ASR 结果是字符串，但 JSON 解析失败，请检查返回结果格式。")
                    logger.error(f"完整 JSON 结果: {json.dumps(result_info, ensure_ascii=False, indent=4)}")
                    return None

            if isinstance(asr_result_json, dict) and 'lattice' in asr_result_json and asr_result_json['lattice']:
                best_result_text = ""
                for item in asr_result_json['lattice']:
                    if 'json_1best' in item and 'st' in item['json_1best'] and 'rt' in item['json_1best']['st']:
                        for rt_item in item['json_1best']['st']['rt']:
                            if 'ws' in rt_item:
                                for ws_item in rt_item['ws']:
                                    if 'cw' in ws_item and ws_item['cw']:
                                        best_result_text += ws_item['cw'][0].get('w', '')
                logger.info("简洁 ASR 结果:")
                logger.info(best_result_text)
                return best_result_text
            else:
                logger.error("无法解析 ASR 结果，请检查 JSON 结构。")
                logger.error(f"完整 JSON 结果: {json.dumps(result_info, ensure_ascii=False, indent=4)}")
                return None
        else:
            logger.error("获取 ASR 结果失败:")
            logger.error(result_info)
            return None

def main(args=None):
    rclpy.init(args=args)
    asr_node = ASR()
    try:
        rclpy.spin(asr_node)
    except KeyboardInterrupt:
        logger.info("ASR 节点关闭")
    finally:
        asr_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
