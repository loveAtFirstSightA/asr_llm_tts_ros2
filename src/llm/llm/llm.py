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
import threading
import requests
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import ollama
from google import genai
import json
import time
from loguru import logger
from llm.sys_prompt import AGENT_SYS_PROMPT

# Configure loguru output
logger.remove()
logger.add(
    sys.stdout,
    format="[{time:YYYY-MM-DD HH:mm:ss.SSS}] [{level}] {message}",
    level="INFO"
)
logger.info('正在使用 loguru 日志系统')

class LLM(Node):

    def __init__(self):
        super().__init__('llm')
        self.asr_subscriber_ = self.create_subscription(
            String, 'asr_result', self.asr_subscriber_callback, 10)
        self.llm_publisher_ = self.create_publisher(String, 'text_to_tts', 10)
        self.declare_parameter('llm_type', 'llm_type')
        self.llm_type_ = self.get_parameter('llm_type').get_parameter_value().string_value
        # command
        self.camera_publisher_ = self.create_publisher(String, 'camera_command', 10)
        self.arm_publisher_ = self.create_publisher(String, 'arm_command', 10)
        # TODO 待调试
        # interface
        self.interface_publisher_ = self.create_publisher(String, 'interface_msg', 10)

    def asr_subscriber_callback(self, msg):
        logger.info('Received message asr result: %s' % msg.data) # Use loguru logger
        thread = threading.Thread(target=self.call_llm_service, args=(msg.data,))
        thread.start()
        # TODO 待调试
        interface_msgs = String()
        interface_msgs = msg
        logger.info('Publishing message to interface: %s' % interface_msgs.data)
        self.interface_publisher_.publish(interface_msgs)

    def call_llm_service(self, asr_result):
        logger.info('Use LLM model type: %s' % self.llm_type_) # Use loguru logger
        if self.llm_type_ == 'deepseak':
            LLM_result = self.deepseak(asr_result)
        elif self.llm_type_ == 'google':
            LLM_result = self.google_gemini(asr_result)
        elif self.llm_type_ == 'baidu':
            LLM_result = self.baidu_qianfan(asr_result)
        logger.info('LLM result: %s' % LLM_result) # Use loguru logger
        parsed_result = self.parse_llm_result_json(LLM_result)
        logger.info('LLM parsed_result: %s ' % parsed_result) # Use loguru logger

        if parsed_result:
            # 可以像字典一样访问解析后的 JSON 对象
            if "function" in parsed_result:
                logger.info("\nFunctions:")
                for func_name_with_params in parsed_result["function"]: # 修改变量名，更清晰
                    # 分割函数名，只保留函数名本身，去除括号和参数
                    func_name = func_name_with_params.split('(')[0]
                    logger.info(f"- {func_name}")
                    # 动态调用解析出的函数
                    if hasattr(self, func_name):
                        func = getattr(self, func_name)
                        func()
                    else:
                        logger.warning(f"Warning: Function '{func_name}' not found in self.")

            if "response" in parsed_result:
                logger.info(f"Response: {parsed_result['response']}")
                # publish tts
                msg = String()
                logger.info('Publishing message tts: %s' % parsed_result['response']) # Use loguru logger
                msg.data = parsed_result['response']
                self.llm_publisher_.publish(msg)
        else:
            logger.info("无法解析 JSON 或未找到 JSON 标记。")

    def deepseak(self, prompt='你好，你是谁？'):
        content = AGENT_SYS_PROMPT + prompt
        logger.info('contents: %s' % content)
        response = ollama.chat(model='deepseek-r1:14b', messages=[
            {
                'role': 'user',
                'content': content,
            },
        ])
        response_content = response['message']['content']
        logger.info('deepseek response: %s' % response_content)

        # 查找关键词 '</think>' 的位置
        think_end_tag = '</think>'
        think_end_index = response_content.find(think_end_tag)

        if think_end_index != -1:
            # 提取 '</think>' 标签后面的所有内容
            extracted_content = response_content[think_end_index + len(think_end_tag):].strip() # strip() 去除前后空白
            logger.info('提取的内容: %s' % extracted_content) # 打印提取的内容，用于调试
            return extracted_content
        else:
            logger.info('未找到关键词 </think>') # 打印未找到关键词的提示，用于调试
            return response_content # 修改: 如果未找到关键词，返回原始的 response 内容, 而不是 None

    def parse_llm_result_json(self, llm_result):
        llm_result = str(llm_result)

        if llm_result.startswith("```json") and llm_result.endswith("```"):
            llm_result = llm_result[7:-3].strip()

        # 删除空格和回车
        llm_result = llm_result.replace(" ", "").replace("\n", "")

        try:
            parsed_json = json.loads(llm_result)
            return parsed_json
        except json.JSONDecodeError as e:
            logger.error(f"JSON 解析错误: {e}")
            logger.error(f"无法解析的 JSON 字符串: {llm_result}")
            return None
        except Exception as e:
            logger.error(f"解析 JSON 时发生错误: {e}")
            return None

    def google_gemini(self, prompt='你好，你是谁？'):
        content = AGENT_SYS_PROMPT + prompt
        client = genai.Client(api_key='AIzaSyBwvNef1hTp_ZP-JAs8an1vvCM8ZtC0kCs')
        logger.info('contents: %s' % content)
        response = client.models.generate_content(
            model="gemini-2.0-flash",
            contents=content
        )
        logger.info('gemini response: %s' % response.text)
        return response.text

    def baidu_qianfan(self, prompt='你好，你是谁？'):
        content = AGENT_SYS_PROMPT + prompt
        headers = {
            'Content-Type': 'application/json',
            'X-Appbuilder-Authorization': 'Bearer bce-v3/ALTAK-Ph5HKAos7Eie98jFnAQdx/66935b2d5d92626966e9d0b5e81c9ddaf0439ef9'
        }
        app_id = "fd963b90-de1c-4926-8575-a86af990fba6"

        # 步骤 1: 调用 conversation 接口初始化会话
        conversation_url = "https://qianfan.baidubce.com/v2/app/conversation"
        conversation_payload = json.dumps({
            "app_id": app_id
        }, ensure_ascii=False)
        conversation_response = requests.request("POST", conversation_url, headers=headers, data=conversation_payload.encode("utf-8"))

        # 尝试从 conversation 接口的响应中提取 conversation_id
        try:
            conversation_data = json.loads(conversation_response.text)
            conversation_id = conversation_data.get("conversation_id")
            if not conversation_id:
                logger.warning("Warning: conversation_id not found in conversation API response.")
                conversation_id = "default_conversation_id" # 使用默认值或根据需要处理
        except json.JSONDecodeError:
            logger.error("Error: Could not decode JSON response from conversation API.")
            conversation_id = "default_conversation_id" # 使用默认值或根据需要处理

        # 步骤 2: 调用 conversation/runs 接口发送 prompt 查询
        runs_url = "https://qianfan.baidubce.com/v2/app/conversation/runs"
        runs_payload = json.dumps({
            "app_id": app_id,
            "query": content,
            "conversation_id": conversation_id,
            "stream": False
        }, ensure_ascii=False)

        runs_response = requests.request("POST", runs_url, headers=headers, data=runs_payload.encode("utf-8"))

        return runs_response.text

    def utils_sleep(self):
        time.sleep(3)
        logger.info("Executing utils_sleep()")

    def utils_camera(self):
        msg = String()
        msg.data = 'take_photo'
        logger.info('Publishing message utils_camera: %s' % msg.data) # 使用 loguru logger
        self.camera_publisher_.publish(msg)
        logger.info("Executing utils_camera() - 拍摄一张图片")
        # 这里添加拍摄图片的具体代码，例如调用摄像头 ROS2 Action/Service

    def utils_set_arm_zero(self):
        msg = String()
        msg.data = 'set_arm_zero'
        logger.info('Publishing message utils_set_arm_zero: %s' % msg.data) # 使用 loguru logger
        self.arm_publisher_.publish(msg)
        logger.info("Executing utils_set_arm_zero() - 机械臂回到零位")
        # 这里添加机械臂回到零位的具体代码，例如调用机械臂控制 ROS2 Action/Service

    def utils_set_arm_rotate(self):
        msg = String()
        msg.data = 'set_arm_rotate'
        logger.info('Publishing message utils_set_arm_rotate: %s' % msg.data) # 使用 loguru logger
        self.arm_publisher_.publish(msg)
        logger.info("Executing utils_set_arm_rotate() - 机械臂回到旋转位置")
        # 这里添加机械臂回到旋转的具体代码，例如调用机械臂控制 ROS2 Action/Service

    def utils_set_arm_rest(self):
        msg = String()
        msg.data = 'set_arm_rest'
        logger.info('Publishing message utils_set_arm_rest: %s' % msg.data) # 使用 loguru logger
        self.arm_publisher_.publish(msg)
        logger.info("Executing utils_set_arm_rest() - 机械臂回到休息位置")
        # 这里添加机械臂回到休息位置的具体代码，例如调用机械臂控制 ROS2 Action/Service

def main(args=None):
    rclpy.init(args=args)
    llm_node = LLM()
    try:
        rclpy.spin(llm_node)
    except KeyboardInterrupt:
        logger.info("LLM node interrupted by user.") # Use loguru logger
    finally:
        llm_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()