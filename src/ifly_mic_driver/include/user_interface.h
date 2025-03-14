/* user_interface_h */
#include "asr_offline_record_sample.h"
/***************************参数配置区域，用户可通过修改这些词来进行 ********************************/
#define whether_print_log 0 //是否打印log
#define TIMEOUT 10 //在客户端获取服务端结果时，超时时间

/******麦克风基础功能参数******/
int PCM_MSG_LEN = 1024; //在录音时会发布音频流,单次发布大小为2048B
bool save_pcm_local = true; //保存音频到本地.
int max_pcm_size = 10240000; //最大为10M,超过10M后自动删除,以节省空间.
/***************************参数配置区域，用户可通过修改这些词来进行 ***************************************
#define _GNU_SOURCE

/********************************[1]　运行效果调试参数****************************************/
int confidence = 15; //置信度阈值，可根据麦克风使用环境进行设置，若容易检测出错，则调大该值。

int time_per_order = 3;//一次命令时长
int max_asr_time = 10;// 录音时长
int awake_count = 5;//一次唤醒，可允许对话的次数
int vol_value = 27;//调整分贝因素
int Invalid_sound = 8;//无效声音次数
int recognize_fail = 4;//识别失败次数
char *LEX_NAME = (char*)"contact";
char *APPID = "bd641c55";//"947d4417"; //APPID
/*************************[2]语法识别资源路径、语法路径、音频文件保存地址等**********************/
//语法相关参数,路径最好设置为绝对地址
char begin[]="fo|/home/wheeltec/SR2.2-HR1.1.3/vvui";//begin+ASR_RES_PATH
char source_path[]="/home/wheeltec/SR2.2-HR1.1.3/vvui";//source_path+GRM_BUILD_PATH等
char *ASR_RES_PATH = (char*)"/bin/config/msc/res/asr/common.jet"; //离线语法识别资源路径，重要，与麦克风及appid绑定
char *GRM_BUILD_PATH = (char*)"/bin/config/msc/res/asr/GrmBuilld";   //构建离线语法识别网络生成数据保存路径
char *GRM_FILE = (char*)"/config/call.bnf";					//构建离线识别语法网络所用的语法文件，用户自修改文件


//录音文件保存的地址,最好设置为绝对地址
#define ORIGINAL_SOUND_PATH "/home/lio/asr_llm_tts_ros2/src/ifly_mic_driver/audio/ori"  
#define DENOISE_SOUND_PATH "/home/lio/asr_llm_tts_ros2/src/ifly_mic_driver/audio/deno"

//资源文件存储地址
#define SYSTEM_PATH "/home/lio/asr_llm_tts_ros2/src/ifly_mic_driver/tmp/system.tar"
#define SYSTEM_CONFIG_PATH "/home/lio/asr_llm_tts_ros2/src/ifly_mic_driver/tmp/config.txt"

// 唤醒词
char awake_words[30] = "你好小小"; //使用的唤醒词


int whether_finised;
struct speech_rec iat;
UserData asr_data;

