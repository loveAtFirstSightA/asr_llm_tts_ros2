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
from rclpy.node import Node
from std_msgs.msg import String
import cv2  # 导入 OpenCV 库
from loguru import logger  # 导入 loguru

# Configure loguru output
logger.remove()
logger.add(
    sys.stdout,
    format="[{time:YYYY-MM-DD HH:mm:ss.SSS}] [{level}] {message}",
    level="INFO"
)
logger.info('正在使用 loguru 日志系统')


class Camera(Node):

    def __init__(self):
        super().__init__('camera')
        self.camera_subscriber_ = self.create_subscription(
            String, 'camera_command', self.camera_subscriber_callback, 10)
        self.cap = cv2.VideoCapture(0) # 初始化摄像头

        if not self.cap.isOpened():
            logger.error("Could not open camera initially!") # Use loguru logger
        else:
            logger.info("Camera opened successfully.") # Use loguru logger


    def camera_subscriber_callback(self, msg):
        logger.info('Received message: %s' % msg.data) # Use loguru logger
        if msg.data == "take_photo":
            # take a photo
            self.take_photo()

    def take_photo(self):
        logger.info('take photo') # Use loguru logger

        if not self.cap.isOpened():
            logger.error("Camera is not open! Attempting to re-open...") # Use loguru logger
            self.cap = cv2.VideoCapture(0) # 尝试重新打开摄像头
            if not self.cap.isOpened():
                logger.error("Failed to re-open camera.") # Use loguru logger
                return
            else:
                logger.info("Camera re-opened successfully.") # Use loguru logger

        # 读取一帧图像
        ret, frame = self.cap.read()

        if not ret:
            logger.error("Could not read frame") # Use loguru logger
            return

        # 保存图像
        photo_path = './photo.jpg' # 可以自定义保存路径和文件名
        cv2.imwrite(photo_path, frame)
        logger.info('Saved photo to %s' % photo_path) # Use loguru logger

        # 注意：  摄像头在 __init__ 中初始化，这里不再释放，保持打开状态
        # self.cap.release()


def main(args=None):
    rclpy.init(args=args)
    camera_node = Camera()
    rclpy.spin(camera_node) # 修改为 spin(camera_node)
    camera_node.cap.release() # 在节点 shutdown 前释放摄像头
    rclpy.shutdown()

if __name__ == '__main__':
    main()