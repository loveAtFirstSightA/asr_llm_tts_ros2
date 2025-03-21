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
from std_msgs.msg import Int64
import ollama
from google import genai
import json
import time
from loguru import logger
import re
import ast
from openai import OpenAI
import jieba
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
        self.declare_parameter('deepseek_model', 'deepseek_model')
        self.deepseek_model_ = self.get_parameter('deepseek_model').get_parameter_value().string_value
        self.llm_publisher_ = self.create_publisher(String, 'text_to_tts', 10)
        self.declare_parameter('llm_type', 'llm_type')
        self.llm_type_ = self.get_parameter('llm_type').get_parameter_value().string_value
        # command
        self.camera_publisher_ = self.create_publisher(String, 'camera_command', 10)
        self.arm_publisher_ = self.create_publisher(String, 'arm_command', 10)
        # TODO 待调试SOCKET
        # interface
        self.interface_publisher_ = self.create_publisher(String, 'interface_msg', 10)
        # TODO 测试语音控制折叠毛巾
        self.arm_command_publisher_ = self.create_publisher(String, 'voice_command', 10)
        # NLP预处理标注位
        self.mini_nlp_ = False
        # 声明NLP预处理词组
        self.action_noun_phrases_stack_towel = ["叠毛巾"]

    def is_sublist(self, sublist, mainlist):
        if not sublist:
            return True
        if not mainlist:
            return False
        for i in range(len(mainlist) - len(sublist) + 1):
            if mainlist[i:i + len(sublist)] == sublist:
                return True
        return False
    
    def asr_subscriber_callback(self, msg):
        logger.info('Received message asr result: %s' % msg.data)
        # NLP预处理 动作名词关键字搜索
        if msg.data:
            text = msg.data
            # 1. 分词 (使用精确模式)
            processed_tokens = list(jieba.cut(text))
            logger.info('Processed tokens (jieba): %s' % processed_tokens)
            # 2. 动作名词关键字搜索 (修改后的逻辑)
            found_actions = False
            for phrase in self.action_noun_phrases_stack_towel:
                phrase_tokens = list(jieba.cut(phrase)) # 对关键字词组使用精确模式
                if self.is_sublist(phrase_tokens, processed_tokens):
                    found_actions = True
                    break
            if found_actions:
                logger.info('Found action noun keywords: %s' % self.action_noun_phrases_stack_towel)
                self.mini_nlp_ = True
                # 发布动作指令 叠毛巾
                self.send_stack_towel()
            else:
                logger.info('No action noun keywords found.')
        
        thread = threading.Thread(target=self.call_llm_service, args=(msg.data,))
        thread.start()
        # TODO 待调试
        interface_msgs = String()
        interface_msgs = msg
        logger.info('Publishing message to interface: %s' % interface_msgs.data)
        self.interface_publisher_.publish(interface_msgs)

    def call_llm_service(self, asr_result):
        logger.info('Use LLM model type: %s' % self.llm_type_)
        # ollama 本机部署模型
        if self.llm_type_ == 'deepseek':
            LLM_result = self.chat_deepseek(asr_result)
            
        # 调用官方api接口
        elif self.llm_type_ == 'deepseek_official':
            LLM_result = self.chat_deepseek_official(asr_result)
            
        # 调用丰巢本地部署接口
        elif self.llm_type_ == 'deepseek_hivebox':
            LLM_result = self.chat_deepseek_hivebox(asr_result)
            
        elif self.llm_type_ == 'google':
            LLM_result = self.chat_google_gemini(asr_result)
            
        elif self.llm_type_ == 'baidu':
            LLM_result = self.chat_baidu_qianfan(asr_result)
            
        logger.info('LLM result: %s' % LLM_result)
        parsed_result = self.parse_llm_result_json(LLM_result)
        logger.info('LLM parsed_result: %s ' % parsed_result)
        
        if parsed_result:
            # 访问解析后的 JSON 对象
            if "function" in parsed_result:
                logger.info("\nFunctions:")
                for func_call in parsed_result["function"]:
                    # 正则匹配函数名和参数
                    match = re.match(r'^(\w+)\((.*)\)$', func_call.strip())
                    if match:
                        func_name = match.group(1)
                        args_str = match.group(2)
                        args = []
                        if args_str:
                            try:
                                # 将参数转换为 Python 对象
                                parsed_args = ast.literal_eval(f'({args_str})')
                                args = list(parsed_args) if isinstance(parsed_args, tuple) else [parsed_args]
                            except (SyntaxError, ValueError) as e:
                                logger.error(f"参数解析失败: '{args_str}' - {e}")
                                continue
                    else:
                        func_name = func_call.strip()
                        args = []
                    
                    logger.info(f"- {func_name}({', '.join(map(str, args))})")
                    if hasattr(self, func_name):
                        try:
                            getattr(self, func_name)(*args)
                        except Exception as e:
                            logger.error(f"调用 {func_name} 失败: {e}")
                    else:
                        logger.warning(f"函数未找到: '{func_name}'")

            if "response" in parsed_result:
                logger.info(f"Response: {parsed_result['response']}")
                msg = String()
                msg.data = parsed_result['response']
                logger.info(f'发布 tts 消息: {msg.data}')
                self.llm_publisher_.publish(msg)
        else:
            logger.info("无法解析 JSON 或未找到 JSON 标记。")

    def chat_deepseek(self, prompt='你好，你是谁？'):
        logger.info("当前模型类型是： deepseek")
        content = AGENT_SYS_PROMPT + prompt
        logger.info('contents: %s' % content)
        response = ollama.chat(model=self.deepseek_model_, messages=[
        # response = ollama.chat(model='deepseek-r1:8b', messages=[
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
    
    def chat_deepseek_official(self, prompt='你好，你是谁？'):
        logger.info("当前模型类型是： deepseek_official")
        content = AGENT_SYS_PROMPT + prompt
        logger.info('contents: %s' % content)
        client = OpenAI(api_key="sk-3c928ce7ca3645a68f0c91f734682460", base_url="https://api.deepseek.com")
        messages = [{"role": "system", "content": AGENT_SYS_PROMPT},
                    {"role": "user", "content": prompt}]
        response = client.chat.completions.create(
            model="deepseek-chat",
            messages=messages,
            response_format={
                'type': 'json_object'
            }
        )
        # print(json.loads(response.choices[0].message.content))
        # 使用 json.dumps 将 Python 字典转换为 JSON 字符串，并指定 ensure_ascii=False 以支持中文
        json_data = json.dumps(json.loads(response.choices[0].message.content), ensure_ascii=False)
        print(json_data)
        return json_data
    
    # TODO 需要链接局域网
    def chat_deepseek_hivebox(self, prompt='你好，你是谁？'):
        logger.info("当前的模型是： deepseek_hivebox（本地化部署）")
        content = AGENT_SYS_PROMPT + prompt
        logger.info('contents: %s' % content)
        BASE_URL = "https://aiapi.fcbox.com/v1/"
        API_SECRET_KEY = "fc-2025032013505854200WDAYD12714A95AD8231C732115BDE"
        """
        其他模型对话
        :param query: 提问
        """
        client = OpenAI(api_key=API_SECRET_KEY, base_url=BASE_URL)
        resp = client.chat.completions.create(
            model="deepseek-r1-distill-llama-70b",
            messages=[
                {"role": "system", "content": AGENT_SYS_PROMPT},
                {"role": "user", "content": prompt}
            ]
        )
        print(resp)
        # 提取 JSON 回复
        if resp.choices and resp.choices[0].message and resp.choices[0].message.content:
            json_reply_str = resp.choices[0].message.content
            # 可以选择直接打印或者返回这个 JSON 字符串
            print("\n提取到的 JSON 回复:")
            print(json_reply_str)
            return json_reply_str
        else:
            print("\n未能提取到 JSON 回复。")
            return None

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

    def chat_google_gemini(self, prompt='你好，你是谁？'):
        content = AGENT_SYS_PROMPT + prompt
        client = genai.Client(api_key='AIzaSyBwvNef1hTp_ZP-JAs8an1vvCM8ZtC0kCs')
        logger.info('contents: %s' % content)
        response = client.models.generate_content(
            model="gemini-2.0-flash",
            contents=content
        )
        logger.info('gemini response: %s' % response.text)
        return response.text

    def chat_baidu_qianfan(self, prompt='你好，你是谁？'):
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
        
        # 打印 conversation 请求的反馈
        print("conversation response:", conversation_response.text)

        # 尝试从 conversation 接口的响应中提取 conversation_id
        try:
            conversation_data = json.loads(conversation_response.text)
            conversation_id = conversation_data.get("conversation_id")
            if not conversation_id:
                logger.warning("Warning: conversation_id not found in conversation API response.")
                conversation_id = "default_conversation_id"  # 使用默认值或根据需要处理
        except json.JSONDecodeError:
            logger.error("Error: Could not decode JSON response from conversation API.")
            conversation_id = "default_conversation_id"  # 使用默认值或根据需要处理

        # 步骤 2: 调用 conversation/runs 接口发送 prompt 查询
        runs_url = "https://qianfan.baidubce.com/v2/app/conversation/runs"
        runs_payload = json.dumps({
            "app_id": app_id,
            "query": content,
            "conversation_id": conversation_id,
            "stream": False
        }, ensure_ascii=False)

        runs_response = requests.request("POST", runs_url, headers=headers, data=runs_payload.encode("utf-8"))
        
        # 打印 runs 请求的反馈
        logger.info("runs response: %s" % runs_response.text)
        
        try:
            # 解析 runs_response 的 JSON 数据
            response_data = json.loads(runs_response.text)
        except json.JSONDecodeError as e:
            print("无法解析 runs_response 的 JSON 数据:", e)
            return None

        # 获取 answer 字段，该字段中包含以 ```json 开头的 JSON 字符串
        answer_str = response_data.get("answer", "")
        if not answer_str:
            print("未找到 answer 字段")
            return None

        # 使用正则表达式提取 ```json 和 ``` 中间的部分
        match = re.search(r"```json\n(.*?)\n```", answer_str, re.DOTALL)
        if match:
            json_block = match.group(1)
            try:
                extracted_data = json.loads(json_block)
                extracted_data_str = json.dumps(extracted_data, ensure_ascii=False)

                logger.info(extracted_data_str)
                return extracted_data_str
            except json.JSONDecodeError as e:
                print("提取的 JSON 数据解析失败:", e)
                return None
        else:
            print("未能匹配到正确的 JSON 格式")
            return None
        
    def utils_sleep(self, second):
        time.sleep(second)
        logger.info("Executing utils_sleep(%ss)" % second)

    def utils_camera(self):
        msg = String()
        msg.data = 'take_photo'
        logger.info('Publishing message utils_camera: %s' % msg.data) # 使用 loguru logger
        self.camera_publisher_.publish(msg)
        logger.info("Executing utils_camera() - 拍摄一张图片")
        # 这里添加拍摄图片的具体代码，例如调用摄像头 ROS2 Action/Service
    
    def utils_go_ahead(self, meters):
        logger.info("Executing utils_go_ahead(%sm)" % meters)
        
    def utils_back(self, meters):
        logger.info("Executing utils_back(%sm)" % meters)
        
    def utils_rotate(self, degree):
        logger.info("Executing utils_rotate(%sdegree)" % degree)

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
    
    def utils_stack_towel(self):
        if self.mini_nlp_:
            logger.info('mini nlp recognize success')
            self.mini_nlp_ = False
            return
        self.send_stack_towel()
        logger.info("Executing utils_stack_towel()")
    
    def send_stack_towel(self):
        msg = String()
        msg.data = 'stack_towel'
        self.arm_command_publisher_.publish(msg)
        logger.info("Send action command %s" % msg.data)

def main(args=None):
    rclpy.init(args=args)
    llm_node = LLM()
    try:
        rclpy.spin(llm_node)
    except KeyboardInterrupt:
        logger.info("LLM node interrupted by user.")
    finally:
        llm_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()