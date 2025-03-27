#!/usr/bin/python3
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
from loguru import logger
from rclpy.node import Node
from std_msgs.msg import String
import webrtcvad
import time
import os

# 配置 loguru 输出到终端（不写入文件）
logger.remove()
logger.add(
    sys.stdout,
    format="[{time:YYYY-MM-DD HH:mm:ss.SSS}] [{level}] {message}",
    level="INFO"
)
logger.info('Using loguru log system')

class VAD(Node):
    def __init__(self):
        super().__init__('vad')
        # 订阅接收命令或PCM文件路径消息
        self.vad_command_subscriber_ = self.create_subscription(
            String, 'vad_command', self.vad_command_subscriber_callback, 10)
        # 发布 VAD 检测结果，通知录音程序停止录音
        self.vad_result_publisher_ = self.create_publisher(String, 'vad_result', 10)
        # 参数声明
        self.declare_parameter('silence_threshold', 1)
        self.silence_threshold_ = self.get_parameter('silence_threshold').get_parameter_value().integer_value
        
        # PCM音频参数
        self.sample_rate = 16000
        self.frame_duration = 30
        self.frame_size = int(self.sample_rate * self.frame_duration / 1000) * 2
        self.vad_mode = 3
        # self.silence_threshold = 3.0
        self.silence_threshold = self.silence_threshold_
        logger.info('默音阈值: %d' % self.silence_threshold)

        self.vad = webrtcvad.Vad(self.vad_mode)
        self.stop_requested_ = False

    def vad_command_subscriber_callback(self, msg):
        data = msg.data.strip()
        if data == 'stop_vad':
            logger.info('Received stop_vad command. Setting stop flag.')
            self.stop_requested_ = True
            return

        pcm_file_path = data
        logger.info(f'Received PCM file path: {pcm_file_path}')

        wait_time = 0
        while not os.path.exists(pcm_file_path) and rclpy.ok():
            logger.info(f"PCM file not yet created, waiting... ({wait_time:.1f}s)")
            time.sleep(0.5)
            wait_time += 0.5
            if self.stop_requested_:
                logger.info("Stop command received during waiting for PCM file.")
                return

        if os.path.exists(pcm_file_path):
            self.stop_requested_ = False
            self.start_detection(pcm_file_path)
        else:
            logger.error(f'PCM file still not found: {pcm_file_path}')

    def start_detection(self, pcm_file_path):
        logger.info(f'Starting VAD detection on file: {pcm_file_path}')
        silence_start_time = None

        try:
            with open(pcm_file_path, 'rb') as f:
                f.seek(0, os.SEEK_END)
                while rclpy.ok() and not self.stop_requested_:
                    current_pos = f.tell()
                    frame = f.read(self.frame_size)
                    
                    if self.stop_requested_:
                        logger.info("Stop command received during detection.")
                        break

                    if len(frame) < self.frame_size:
                        for _ in range(int(self.frame_duration / 10)):
                            if self.stop_requested_:
                                logger.info("Stop command received while waiting for data.")
                                break
                            time.sleep(0.01)
                        if self.stop_requested_:
                            break
                        f.seek(current_pos)
                        continue

                    if self.vad.is_speech(frame, self.sample_rate):
                        # 可选：logger.info("Speech detected in current frame.")
                        silence_start_time = None
                    else:
                        # 可选：logger.info("Silence detected in current frame.")
                        if silence_start_time is None:
                            silence_start_time = time.time()
                        else:
                            elapsed = time.time() - silence_start_time
                            # 可选：logger.info(f"Silence duration: {elapsed:.2f} seconds.")
                            if elapsed >= self.silence_threshold:
                                logger.info("Silence threshold exceeded. Stopping detection.")
                                break
        except Exception as e:
            logger.error(f"Exception in start_detection: {e}")
        
        self.stop_detection()

    def stop_detection(self):
        logger.info('stop_detection')
        msg = String()
        msg.data = 'stop_record'
        logger.info(f'Publishing VAD result: {msg.data}')
        self.vad_result_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VAD()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
