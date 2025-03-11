/*******************************************************
 This contents of this file may be used by anyone
 for any reason without any conditions and may be
 used as a starting point for your own applications
 which use HIDAPI.
********************************************************/
#include <stdio.h>
#include <wchar.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <locale.h>
#include <errno.h>
#include <unistd.h>
/* Unix */
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/utsname.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdbool.h>
#include "hidapi.h"
#include "cJSON.h"
/* GNU / LibUSB */
#include <libusb.h>
#include <iconv.h>
#include <sys/select.h>
#define _GNU_SOURCE
#define DENOISE_SOUND_PATH "../audio/mic_demo_vvui_deno.pcm"
#define ORIGINAL_SOUND_PATH "../audio/mic_demo_vvui_ori.pcm"
#define SYSTEM_PATH "../tmp/system.tar"
#define SYSTEM_CONFIG_PATH "../tmp/config.txt"

char awake_words[30] = "你好小微";

int isnumber(char *a, int count_need)
{
	int len = strlen(a);
	if (len > count_need)
	{
		return -1;
	}
	int j = 0;
	for (int i = 0; i < len; i++)
	{
		if (a[i] <= 57 && a[i] >= 48)
		{
			j++;
		} 
	}
	if (j == len)
	{
		return 0;
	}
	else
	{
		return -1;
	}
}

int business_proc_callback(business_msg_t businessMsg)
{
	int res = 0;
	char fileName[] = DENOISE_SOUND_PATH;
	char fileName_ori[] = ORIGINAL_SOUND_PATH;
	static int index = 0;
	unsigned char buf[4096];
	switch (businessMsg.modId)
	{
	case 0x01:
		if (businessMsg.msgId == 0x01)
		{
			unsigned char key[] = "errcode";
			int status = whether_set_succeed(businessMsg.data, key);
			if (status == 0)
			{
				//printf("\n>>>>>您已开启录音\n");
			}
		}
		else if (businessMsg.msgId == 0x02)
		{
			char fileName[] = DENOISE_SOUND_PATH;
			get_denoised_sound(fileName, businessMsg.data);
		}
		else if (businessMsg.msgId == 0x03)
		{
			unsigned char key[] = "errcode";
			int status = whether_set_succeed(businessMsg.data, key);
			if (status == 0)
			{
				//printf("\n>>>>>您已停止录音\n");
			}
		}
		else if (businessMsg.msgId == 0x04)
		{
			unsigned char key[] = "errcode";
			int status = whether_set_succeed(businessMsg.data, key);
			if (status == 0)
			{
				//printf("\n>>>>>开/关原始音频成功\n");
			}
		}
		else if (businessMsg.msgId == 0x05)
		{
			unsigned char key[] = "errcode";
			int status = whether_set_succeed(businessMsg.data, key);
			if (status == 0)
			{
				printf("\n>>>>>设置主麦克风和灯光成功,升级版本不推荐使用\n");
			}
		}
		else if (businessMsg.msgId == 0x06)
		{
			char fileName_ori[] = ORIGINAL_SOUND_PATH;
			get_original_sound(fileName_ori, businessMsg.data);
		}
		else if (businessMsg.msgId == 0x07)
		{
			unsigned char key2[] = "beam";
			int major_id = whether_set_succeed(businessMsg.data, key2);
			major_mic_id = major_id;
			printf("\n>>>>>主麦克风id为%d\n", major_mic_id);
		}
		else if (businessMsg.msgId == 0x08)
		{
			unsigned char key[] = "errcode";
			int status = whether_set_succeed(businessMsg.data, key);
			if (status == 0)
			{
				printf("\n>>>>>设置主麦克风成功\n");
			}
		}
		else if (businessMsg.msgId == 0x09)
		{
			unsigned char key[] = "errcode";
			int status = whether_set_succeed(businessMsg.data, key);
			if (status == 0)
			{
				printf("\n>>>>>设置灯光成功\n");
			}
		}
		break;
	case 0x02:
		if (businessMsg.msgId == 0x01)
		{
			unsigned char key1[] = "beam";
			unsigned char key2[] = "angle";
			major_mic_id = get_awake_mic_id(businessMsg.data, key1);
			mic_angle = get_awake_mic_angle(businessMsg.data, key2);
			if (major_mic_id <= 5 && major_mic_id >= 0 && mic_angle <= 360 && mic_angle >= 0)
			{
				if_awake = 1;
				led_id = get_led_based_angle(mic_angle);
				int ret1 = set_major_mic_id(major_mic_id);
				int ret2 = set_target_led_on(led_id);
				if (ret1 == 0 && ret2 == 0)
				{
					printf("\n>>>>>第%d个麦克风被唤醒\n", major_mic_id);
					printf("\n>>>>>唤醒角度为:%d\n", mic_angle);
					printf("\n>>>>>已点亮%d灯\n", led_id);
				}
			}
		}
		else if (businessMsg.msgId == 0x08)
		{
			unsigned char key1[] = "errstring";
			int result = whether_set_awake_word(businessMsg.data, key1);
			if (result==0)
			{
				printf("\n>>>>>唤醒词设置成功\n");
			}
			else if (result==-2)
			{
				printf("\n>>>>>唤醒词设置失败\n");
			}
		}

		break;
	case 0x03:
		if (businessMsg.msgId == 0x01)
		{
			unsigned char key[] = "status";
			int status = whether_set_succeed(businessMsg.data, key);
			char protocol_version[40]; 
			int ret = get_protocol_version(businessMsg.data,protocol_version);
			printf(">>>>>麦克风%s,软件版本为:%s,协议版本为:%s\n", (status == 0 ? "正常工作" : "正在启动"),get_software_version(),protocol_version);
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
		if (businessMsg.msgId == 0x01)
		{
			whether_set_resource_info(businessMsg.data);
		}
		else if (businessMsg.msgId == 0x03) //文件接收结果
		{
			whether_set_resource_info(businessMsg.data);
		}
		else if (businessMsg.msgId == 0x04) //查看设备升级结果
		{
			whether_upgrade_succeed(businessMsg.data);
		}
		else if (businessMsg.msgId == 0x05) //下发文件
		{
			char fileName[] = SYSTEM_PATH;
			send_resource(businessMsg.data, fileName, 1);
		}
		else if (businessMsg.msgId == 0x08) //获取升级配置文件
		{
			printf("config.json: %s\n", businessMsg.data);
		}
		break;

	default:
		break;
	}
	return 0;
}

void Direction()
{
	printf("demo示例为输入命令，调用对应的函数，可以简单熟悉麦克风相关的基本功能，如:\n");
	printf("1命令，获取麦克风的工作状态，若麦克风未启动，则麦克风会自动启动至工作状态\n");
	printf("2命令，开启降噪音频录音功能，执行该命令前请务必保持设备为唤醒状态\n");
	printf("3命令，停止降噪音频录音功能,在指定的路径中保存音频文件\n");
	printf("4命令，开启原始音频录音功能，执行该命令前请务必保持设备为唤醒状态\n");
	printf("5命令，停止原始音频录音功能,在指定的路径保存音频文件\n");
	printf("6命令，同时开启降噪音频和原始音频录音功能，执行该命令前请务必保持设备为唤醒状态\n");
	printf("7命令，同时停止降噪音频和原始音频录音功能，在指定的路径保存音频文件\n");
	printf("8命令，设置主麦id\n");
	printf("9命令, 获取主麦克风id\n");
	printf("10命令,点亮和关闭灯光\n");
	printf("11命令,设置自定义唤醒词\n");
}
int main(int argc, char *argv[])
{
	hid_device *handle;
	handle = hid_open();
	if (!handle)
	{
		printf("\n>>>>>无法打开麦克风设备，请检测设备连接\n");
		return -1;
	}
	printf("\n>>>>>成功打开麦克风设备\n");


	protocol_proc_init(send_to_usb_device, recv_from_usb_device, business_proc_callback, err_proc);

	Direction();

	while (1)
	{
		printf("\n>>>>>请输入指令\n:");
		char a[10];
		int cmd = 0;
		scanf("%s", &a);
		int ret = isnumber(a, 2);
		if (ret != 0)
		{
			printf("\n>>>>>指令有误，请重新输入");
			continue;
		}
		cmd = atoi(a);
		if (cmd == 0)
		{
			break;
		}
		else
		{
			switch (cmd)
			{
			case 1:
			{
				get_system_status();
				sleep(3);
				if (!is_boot)
				{
					printf(">>>>>开机中，请稍等！\n");
				}
				while (!is_boot)
				{
					if (is_reboot)
					{
						break;
					}
				}
				printf(">>>>>开机成功！\n");
				set_awake_word(awake_words);
				sleep(2.0);

				break;
			}
			case 2:
			{
				/*在获取降噪音频前，先获取下有没有设置主麦，若没有主麦，则降噪音频无法获取．*/
				if (major_mic_id>5 || major_mic_id<0)
				{
					printf("\n>>>>>您还未唤醒或设置主麦方向，请唤醒或设置后再进行录音操作\n");
					break;
				}
				start_to_record_denoised_sound();
				sleep(1);
				break;
			}
			case 3:
			{
				finish_to_record_denoised_sound();
				sleep(1);
				break;
			}
			case 4:
			{
				start_to_record_original_sound();
				sleep(1);
				break;
			}
			case 5:
			{
				finish_to_record_original_sound();
				sleep(1);
				break;
			}
			case 6:
			{
				/*在获取降噪音频前，先获取下有没有设置主麦，若没有主麦，则降噪音频无法获取．*/
				if (major_mic_id>5 || major_mic_id<0)
				{
					printf("\n>>>>>您还未唤醒或设置主麦方向，请唤醒或设置后再进行录音操作\n");
					break;
				}
				start_to_record_denoised_sound();
				start_to_record_original_sound();
				sleep(1);
				break;
			}
			case 7:
			{
				finish_to_record_denoised_sound();
				finish_to_record_original_sound();
				sleep(1);
				break;
			}
			case 8:
			{
				int mic_id = 0;
				char mic[10];
				printf("\n>>>>>请输入主麦克风id:");
				while (1)
				{
					scanf("%s", &mic);
					int ret = isnumber(mic, 1);
					if (ret != 0 || atoi(mic) > 5 || atoi(mic) < 0)
					{
						printf("\n>>>>>请重新输入主麦克风id:");
					}
					else
					{
						break;
					}
				}
				mic_id = atoi(mic);
				int led_id = get_led_based_mic_id(mic_id);
				if (led_id != -1)
				{
					int ret1 = set_major_mic_id(mic_id);
					if (ret1 != 0)
					{
						printf("\n>>>>>主麦设置失败,请检查主麦id:");
					}
					int ret2 = set_target_led_on(led_id);
					if (ret2 != 0)
					{
						printf("\n>>>>>灯光设置失败,请检查灯光id:");
					}
				}
				sleep(1);
				break;
			}
			case 9:
			{
				get_major_mic_id();//发送获取主麦方向的请求，其结果会在回调函数中获取．
				sleep(1);
				break;
			}
			case 10:
			{
				int led_id = 0;
				char led[10];
				printf("\n>>>>>请输入要点亮的灯id,特别地99表示关闭灯光:");
				while (1)
				{
					scanf("%s", &led);
					int ret = isnumber(led, 2);
					if (ret != 0 || !(atoi(led) <= 11 && atoi(led) >= 0 || atoi(led) == 99))
					{
						printf("\n>>>>>请重新输入灯光id:");
					}
					else
					{
						break;
					}
				}
				led_id = atoi(led);
				int ret = set_target_led_on(led_id);
				if (ret != 0)
				{
					printf("\n>>>>>灯光设置失败,将检测灯光id:");
				}
				sleep(1);
				break;
			}
			case 11:
			{
				char word[30];
				printf("\n>>>>>请输入唤醒词,4-6个汉字\n:");
				while (1)
				{
					scanf("%s", word);
					if (strlen(word) < 12 || strlen(word) > 18)
					{
						printf("\n>>>>>唤醒词不符合要求,请重新输入\n:");
					}
					else
					{
						break;
					}
				}
				set_awake_word(word);
				sleep(1);
				break;
			}
			default:
			{
				printf("\n>>>>>指令有误，请重新输入:");
				sleep(1);
				break;
			}
			}
		}
	}

    finish_to_record_denoised_sound();
	finish_to_record_original_sound();
    sleep(1);
	hid_close();
	printf("\n>>>>>退出测试用例\n");
	return 0;
}
