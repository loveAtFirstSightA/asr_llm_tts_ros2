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

from asr import seve_file
import requests
import datetime
import hashlib
import base64
import hmac
import json
import os
import re

path_pwd = os.path.split(os.path.realpath(__file__))[0]
os.chdir(path_pwd)


# 创建和查询
class get_result(object):
    def __init__(self, appid, apikey, apisecret, file_path): # 修改: 接受 file_path 参数
        # 以下为POST请求
        self.Host = "ost-api.xfyun.cn"
        self.RequestUriCreate = "/v2/ost/pro_create"
        self.RequestUriQuery = "/v2/ost/query"
        # 设置url
        if re.match("^\d", self.Host):
            self.urlCreate = "http://" + self.Host + self.RequestUriCreate
            self.urlQuery = "http://" + self.Host + self.RequestUriQuery
        else:
            self.urlCreate = "https://" + self.Host + self.RequestUriCreate
            self.urlQuery = "https://" + self.Host + self.RequestUriQuery
        self.HttpMethod = "POST"
        self.APPID = appid
        self.Algorithm = "hmac-sha256"
        self.HttpProto = "HTTP/1.1"
        self.UserName = apikey
        self.Secret = apisecret
        self.file_path = file_path # 修改: 存储 file_path 为实例属性

        # 设置当前时间
        cur_time_utc = datetime.datetime.utcnow()
        self.Date = self.httpdate(cur_time_utc)
        # 设置测试音频文件
        self.BusinessArgsCreate = {
            "language": "zh_cn",
            "accent": "mandarin",
            "domain": "pro_ost_ed",
            # "callback_url": "http://IP:端口号/xxx/"
        }

    def img_read(self, path):
        with open(path, 'rb') as fo:
            return fo.read()

    def hashlib_256(self, res):
        m = hashlib.sha256(bytes(res.encode(encoding='utf-8'))).digest()
        result = "SHA-256=" + base64.b64encode(m).decode(encoding='utf-8')
        return result

    def httpdate(self, dt):
        """
        Return a string representation of a date according to RFC 1123
        (HTTP/1.1).
        The supplied date must be in UTC.
        """
        weekday = ["Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun"][dt.weekday()]
        month = ["Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep",
                 "Oct", "Nov", "Dec"][dt.month - 1]
        return "%s, %02d %s %04d %02d:%02d:%02d GMT" % (weekday, dt.day, month,
                                                        dt.year, dt.hour, dt.minute, dt.second)

    def generateSignature(self, digest, uri):
        signature_str = "host: " + self.Host + "\n"
        signature_str += "date: " + self.Date + "\n"
        signature_str += self.HttpMethod + " " + uri \
                         + " " + self.HttpProto + "\n"
        signature_str += "digest: " + digest
        signature = hmac.new(bytes(self.Secret.encode('utf-8')),
                             bytes(signature_str.encode('utf-8')),
                             digestmod=hashlib.sha256).digest()
        result = base64.b64encode(signature)
        return result.decode(encoding='utf-8')

    def init_header(self, data, uri):
        digest = self.hashlib_256(data)
        sign = self.generateSignature(digest, uri)
        auth_header = 'api_key="%s",algorithm="%s", ' \
                      'headers="host date request-line digest", ' \
                      'signature="%s"' \
                      % (self.UserName, self.Algorithm, sign)
        headers = {
            "Content-Type": "application/json",
            "Accept": "application/json",
            "Method": "POST",
            "Host": self.Host,
            "Date": self.Date,
            "Digest": digest,
            "Authorization": auth_header
        }
        return headers

    def get_create_body(self, fileurl):
        post_data = {
            "common": {"app_id": self.APPID},
            "business": self.BusinessArgsCreate,
            "data": {
                "audio_src": "http",
                "audio_url": fileurl,
                "encoding": "raw"
            }
        }
        body = json.dumps(post_data)
        return body

    def get_query_body(self, task_id):
        post_data = {
            "common": {"app_id": self.APPID},
            "business": {
                "task_id": task_id,
            },
        }
        body = json.dumps(post_data)
        return body

    def call(self, url, body, headers):

        try:
            response = requests.post(url, data=body, headers=headers, timeout=8)
            status_code = response.status_code
            interval = response.elapsed.total_seconds()
            if status_code != 200:
                info = response.content
                return info
            else:
                resp_data = json.loads(response.text)
                return resp_data
        except Exception as e:
            print("Exception ：%s" % e)

    def task_create(self, fileurl): # 修改: 接受 fileurl 参数
        body = self.get_create_body(fileurl)
        headers_create = self.init_header(body, self.RequestUriCreate)
        task_id = self.call(self.urlCreate, body, headers_create) # 修改: 使用 self 调用
        print(task_id)
        return task_id

    def task_query(self, task_id):
        if task_id:
            body = self.get_create_body("")  # 修改: 传递空字符串作为 fileurl 参数
            query_body = self.get_query_body(task_id)
            headers_query = self.init_header(body, self.RequestUriQuery)
            result = self.call(self.urlQuery, query_body, headers_query)
            return result

    def get_fileurl(self):
        # 文件上传
        api = seve_file.SeveFile(app_id=self.APPID, api_key=self.UserName, api_secret=self.Secret, upload_file_path=self.file_path) # 修改: 使用 self 属性
        file_total_size = os.path.getsize(self.file_path) # 修改: 使用 self 属性
        if file_total_size < 31457280:
            print("-----不使用分块上传-----")
            fileurl = api.gene_params('/upload')['data']['url']
        else:
            print("-----使用分块上传-----")
            fileurl = api.gene_params('/mpupload/upload')
        return fileurl

    def get_result(self):
        # 获取文件 url
        fileurl = self.get_fileurl() # 修改: 使用 self 调用
        # 创建订单
        print("\n------ 创建任务 -------")
        task_create_result = self.task_create(fileurl) # 修改: 传递 fileurl 参数, 使用 self 调用
        if isinstance(task_create_result, dict) and 'data' in task_create_result and 'task_id' in task_create_result['data']:
            task_id = task_create_result['data']['task_id']
        else:
            print("创建任务失败或返回异常:", task_create_result) # 打印错误信息
            return task_create_result # 返回错误信息

        # 查询任务
        print("\n------ 查询任务 -------")
        print("任务转写中······")
        while True:
            result = self.task_query(task_id) # 修改: 使用 self 调用
            if isinstance(result, dict) and result['data']['task_status'] != '1' and result['data'][
                'task_status'] != '2':
                print("转写完成···\n", json.dumps(result, ensure_ascii=False))
                return result # 修改: 返回 result
            elif isinstance(result, bytes):
                print("发生错误···\n", result)
                return result # 修改: 返回 result
            # 可以添加 sleep 避免频繁查询，例如 time.sleep(1)

if __name__ == '__main__':
    # 输入讯飞开放平台的appid，secret、key和文件路径
    appid = "xxxx" # 替换为你的 APPID
    apikey = "xxxxxxx" # 替换为你的 APIKey
    apisecret = "xxxxxxx" # 替换为你的 APISecret
    file_path = r".\audio\audio_sample_little.wav" # 音频文件路径，请确保文件存在


    gClass = get_result(appid, apikey, apisecret, file_path) # 修改: 传递 file_path 参数
    result = gClass.get_result() # 修改: 获取返回结果
    if isinstance(result, dict) and 'data' in result and 'result' in result['data']:
        asr_result = result['data']['result']
        print("\n最终 ASR 结果:")
        print(asr_result)
    else:
        print("\n获取最终 ASR 结果失败:")
        print(result) # 打印错误信息