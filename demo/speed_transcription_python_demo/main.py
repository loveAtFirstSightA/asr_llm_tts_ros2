#!/bin/bash/python3
# -*- coding=utf-8 -*-

import ost_fast
import json  # 导入 json 模块，用于解析 JSON 字符串

def main():
    # 替换为你的实际参数
    appid = "6f20e2c2" # 替换为你的 APPID
    apikey = "fc5fc33f5d17487a93263120e7c6d393" # 替换为你的 APIKey
    apisecret = "NzMwZTYwYTY0MWZkNjYyMGM0YjJjMjE4" # 替换为你的 APISecret
    file_path = "/home/lio/asr_llm_tts_ros2/src/ifly_mic_driver/audio/vvui_deno.pcm" # 音频文件路径，请确保文件存在

    # 实例化 get_result 类, 传递 file_path
    asr_instance = ost_fast.get_result(appid, apikey, apisecret, file_path)

    # 获取 ASR 结果
    result_info = asr_instance.get_result()

    # 检查 result_info 是否成功获取结果，并提取 ASR 文本
    if isinstance(result_info, dict) and 'data' in result_info and 'result' in result_info['data']:
        asr_result_json = result_info['data']['result'] # 获取 JSON 格式的 ASR 结果

        # 尝试解析 JSON 字符串，如果 result_info['data']['result'] 是字符串格式的 JSON
        if isinstance(asr_result_json, str):
            try:
                asr_result_json = json.loads(asr_result_json)
            except json.JSONDecodeError:
                print("ASR 结果是字符串，但 JSON 解析失败，请检查返回结果格式。")
                print("完整 JSON 结果:\n", json.dumps(result_info, ensure_ascii=False, indent=4)) # 打印完整 JSON 便于调试
                return

        # 提取文字结果
        if isinstance(asr_result_json, dict) and 'lattice' in asr_result_json and asr_result_json['lattice']:
            lattice_data = asr_result_json['lattice']
            best_result_text = ""
            for item in lattice_data: # 遍历 lattice 列表
                if 'json_1best' in item and 'st' in item['json_1best'] and 'rt' in item['json_1best']['st']:
                    rt_list = item['json_1best']['st']['rt']
                    for rt_item in rt_list: # 遍历 rt 列表 (通常只有一个元素，但为了健壮性遍历一下)
                        if 'ws' in rt_item:
                            ws_list = rt_item['ws']
                            for ws_item in ws_list: # 遍历 ws 列表 (词语列表)
                                if 'cw' in ws_item and ws_item['cw']:
                                    cw_list = ws_item['cw']
                                    first_cw = cw_list[0] # 取第一个候选词
                                    if 'w' in first_cw:
                                        best_result_text += first_cw['w'] # 拼接词语

            print("简洁 ASR 结果:")
            print(best_result_text)
        else:
            print("无法解析 ASR 结果，请检查 JSON 结构。")
            print("完整 JSON 结果:\n", json.dumps(result_info, ensure_ascii=False, indent=4)) # 打印完整 JSON 便于调试

    else:
        print("获取 ASR 结果失败:")
        print(result_info) # 打印错误信息或者其他返回内容

if __name__ == '__main__':
    main()