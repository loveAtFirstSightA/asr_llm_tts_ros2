/* user_interface_h */
#include "asr_offline_record_sample.h"
#define whether_print_log 0 //是否打印log

/******麦克风基础功能参数******/
int PCM_MSG_LEN = 1024; //在录音时会发布音频流,单次发布大小为2048B
bool save_pcm_local = true; //保存音频到本地.
int max_pcm_size = 10240000; //最大为10M,超过10M后自动删除,以节省空间.

char awake_words[30] = "你好小微"; //使用的唤醒词

//语法相关参数,路径最好设置为绝对地址
char begin[]="fo|/home/wheeltec/SR2.2-HR1.1.3/vvui";//begin+ASR_RES_PATH
char source_path[]="/home/wheeltec/SR2.2-HR1.1.3/vvui";//source_path+GRM_BUILD_PATH等

//录音文件保存的地址,最好设置为绝对地址
#define ORIGINAL_SOUND_PATH "/audio/record_online.pcm"  
#define DENOISE_SOUND_PATH "/audio/record_online_deno.pcm"
//资源文件存储地址
#define SYSTEM_PATH "/tmp/system.tar"
#define SYSTEM_CONFIG_PATH "/tmp/config.txt"
