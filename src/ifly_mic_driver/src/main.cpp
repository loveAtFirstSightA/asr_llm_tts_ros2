/*
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
 */


#include <memory>
#include <thread>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "ifly_mic_driver/ifly_mic_driver.hpp"

// sdk
#include "hidapi.h"
#include "user_interface.h"

// 操作系统文件
#include <filesystem>

int business_proc_callback(business_msg_t business_msg)
{
	int res = 0;
	char fileName[] = DENOISE_SOUND_PATH;
	char fileName_ori[] = ORIGINAL_SOUND_PATH;
	static int index = 0;
	unsigned char buf[4096];
	switch (business_msg.modId)
	{
	case 0x01:
		if (business_msg.msgId == 0x01)
		{
			unsigned char key[] = "errcode";
			int status = whether_set_succeed(business_msg.data, key);
			if (status == 0)
			{
				//printf("\n>>>>>您已开启录音\n");
			}
		}
		else if (business_msg.msgId == 0x02)
		{
			char fileName[] = DENOISE_SOUND_PATH;
			get_denoised_sound(fileName, business_msg.data);
		}
		else if (business_msg.msgId == 0x03)
		{
			unsigned char key[] = "errcode";
			int status = whether_set_succeed(business_msg.data, key);
			if (status == 0)
			{
				//printf("\n>>>>>您已停止录音\n");
			}
		}
		else if (business_msg.msgId == 0x04)
		{
			unsigned char key[] = "errcode";
			int status = whether_set_succeed(business_msg.data, key);
			if (status == 0)
			{
				//printf("\n>>>>>开/关原始音频成功\n");
			}
		}
		else if (business_msg.msgId == 0x05)
		{
			unsigned char key[] = "errcode";
			int status = whether_set_succeed(business_msg.data, key);
			if (status == 0)
			{
				printf("\n>>>>>设置主麦克风和灯光成功,升级版本不推荐使用\n");
			}
		}
		else if (business_msg.msgId == 0x06)
		{
            // 通过修改ORIGINAL_SOUND_PATH可调整路径和文件名称
			char fileName_ori[] = ORIGINAL_SOUND_PATH;
			get_original_sound(fileName_ori, business_msg.data);
		}
		else if (business_msg.msgId == 0x07)
		{
			unsigned char key2[] = "beam";
			int major_id = whether_set_succeed(business_msg.data, key2);
			major_mic_id = major_id;
            spdlog::info(">>>>主麦克风id为{}", major_mic_id);
		}
		else if (business_msg.msgId == 0x08)
		{
			unsigned char key[] = "errcode";
			int status = whether_set_succeed(business_msg.data, key);
			if (status == 0)
			{
                spdlog::info(">>>>设置主麦克风成功");
			}
		}
		else if (business_msg.msgId == 0x09)
		{
			unsigned char key[] = "errcode";
			int status = whether_set_succeed(business_msg.data, key);
			if (status == 0)
			{
                spdlog::info(">>>>设置灯光成功");
			}
		}

		break;

	case 0x02:
		if (business_msg.msgId == 0x01)
		{
			unsigned char key1[] = "beam";
			unsigned char key2[] = "angle";
			major_mic_id = get_awake_mic_id(business_msg.data, key1);
			mic_angle = get_awake_mic_angle(business_msg.data, key2);
			if (major_mic_id <= 5 && major_mic_id >= 0 && mic_angle <= 360 && mic_angle >= 0)
			{
				if_awake = 1;
				led_id = get_led_based_angle(mic_angle);
				int ret1 = set_major_mic_id(major_mic_id);
				int ret2 = set_target_led_on(led_id);
				if (ret1 == 0 && ret2 == 0)
				{
                    spdlog::info(">>>>第 {} 个麦克风被唤醒, 唤醒角度为: {}, 已点亮 {} 灯",
                        major_mic_id, mic_angle, led_id);
                    // 如果被唤醒 这里有反馈
                    // 此处可添加被唤醒后的逻辑代码
                    
				}
			}
		}
		else if (business_msg.msgId == 0x08)
		{
			unsigned char key1[] = "errstring";
			int result = whether_set_awake_word(business_msg.data, key1);
			if (result==0)
			{
                spdlog::info(">>>>唤醒词设置成功");
			}
			else if (result==-2)
			{
                spdlog::info(">>>>唤醒词设置失败");
			}
		}

		break;

	case 0x03:
		if (business_msg.msgId == 0x01)
		{
			unsigned char key[] = "status";
			int status = whether_set_succeed(business_msg.data, key);
			char protocol_version[40]; 
			int ret = get_protocol_version(business_msg.data,protocol_version);
            spdlog::info(">>>>麦克风 {}, 软件版本为: {}, 协议版本为: {}",
                (status == 0 ? "正常工作" : "正在启动"), get_software_version(), protocol_version);
			if (status == 1)
			{
				char *fileName = SYSTEM_CONFIG_PATH;
				send_resource_info(fileName, 0);
			}
			else
			{
				is_boot = 1;
			}
			
		}

		break;

	case 0x04:
		if (business_msg.msgId == 0x01)
		{
			whether_set_resource_info(business_msg.data);
		}
		else if (business_msg.msgId == 0x03) //文件接收结果
		{
			whether_set_resource_info(business_msg.data);
		}
		else if (business_msg.msgId == 0x04) //查看设备升级结果
		{
			whether_upgrade_succeed(business_msg.data);
		}
		else if (business_msg.msgId == 0x05) //下发文件
		{
			char fileName[] = SYSTEM_PATH;
			send_resource(business_msg.data, fileName, 1);
		}
		else if (business_msg.msgId == 0x08) //获取升级配置文件
		{
            spdlog::info("config.json: {}", reinterpret_cast<const char*>(business_msg.data));
		}
		break;

	default:
		break;
	}
	return 0;
}

// 查询设备是否开机
int _get_boot(void)
{
    return is_boot;
}
// 查询设备是否被唤醒
int _get_awake(void)
{
    return if_awake;
}
// 设置设备唤醒状态 
void _set_awake(int status)
{
    if_awake = status;
}

void _start_to_record_denoised_sound(void)
{
    // 清理旧文件
    if (std::filesystem::exists(DENOISE_SOUND_PATH)) {
        std::filesystem::remove(DENOISE_SOUND_PATH);
        spdlog::info(">>>>删除历史文件: {}", DENOISE_SOUND_PATH);
    }
    start_to_record_denoised_sound();
}

void _finish_to_record_denoised_sound(void)
{
    finish_to_record_denoised_sound();
}

void _start_to_record_original_sound(void)
{
    // 清理旧文件
    if (std::filesystem::exists(ORIGINAL_SOUND_PATH)) {
        std::filesystem::remove(ORIGINAL_SOUND_PATH);
        spdlog::info(">>>>删除历史文件: {}", ORIGINAL_SOUND_PATH);
    }
    start_to_record_original_sound();
}

void _finish_to_record_original_sound(void)
{
    finish_to_record_original_sound();
}

void _start_to_record_denoised_original_sound(void)
{
    _start_to_record_denoised_sound();
    _start_to_record_original_sound();
}

void _finish_to_record_denoised_original_sound(void)
{
    finish_to_record_denoised_sound();
    finish_to_record_original_sound();
}

int _get_led_based_angle(int mic_angle)
{
    get_led_based_angle(mic_angle);
}

int _get_led_based_mic_id(int mic_id)
{
    get_led_based_mic_id(mic_id);
}

void turn_off_all_led(void)
{
    set_target_led_on(99);
}

int _set_target_led_on(int led_id)
{
    set_target_led_on(led_id);
}

int _get_major_mic_id(void)
{
    return get_major_mic_id();
}

void _set_major_mic_id(int id)
{
    set_major_mic_id(id);
}

void _set_awake_word(const char *awake_words)
{
	set_awake_word(awake_words);
}


int main(int argc, char * argv[])
{
    // 初始化麦克风
    hid_device *handle;
	handle = hid_open();
	if (!handle) {
        spdlog::error(">>>>无法打开麦克风设备，请检测设备连接");
        spdlog::info(">>>>关闭程序");
		return -1;
	}
    spdlog::info(">>>>成功打开麦克风设备");
	protocol_proc_init(send_to_usb_device, recv_from_usb_device, business_proc_callback, err_proc);
    spdlog::info(">>>>麦克风初始化完成, 等待系统开机");
    // 等待麦克风准备就绪
    // 获取麦克风的工作状态，若麦克风未启动，则麦克风会自动启动至工作状态
    get_system_status();
	sleep(3);
    if (!is_boot) {
        spdlog::info(">>>>开机中，请稍等！");
    }
    while (!is_boot) {
        if (is_reboot) {
            break;
        }
    }
    spdlog::info(">>>>开机成功！");
    set_awake_word(awake_words);
    // spdlog::info("设置唤醒词成功，请使用【{}】唤醒我", awake_words);
    sleep(1);
    // 添加led循环点亮表示开机成功，熄灭等待唤醒
    // 先熄灭
    turn_off_all_led();
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 12; j++) {
            set_target_led_on(j);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            turn_off_all_led();
        }           
    }
    // 关闭所有led 等待用户唤醒后点亮led
    turn_off_all_led();

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ifly_mic_driver::IflyMicDriver>());
    rclcpp::shutdown();
    return 0;
}
