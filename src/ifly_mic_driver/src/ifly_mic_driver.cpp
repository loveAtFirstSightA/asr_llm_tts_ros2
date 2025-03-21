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


#include "ifly_mic_driver/ifly_mic_driver.hpp"
// #include "user_interface.h"
#include <iostream>
#include <ctime>
#include <iomanip>
#include <sstream>

extern int _get_boot(void);
extern int _get_awake(void);
extern void _set_awake(int status);
extern void _start_to_record_denoised_sound(std::string timestamp);
extern void _finish_to_record_denoised_sound(void);
extern void _start_to_record_original_sound(std::string timestamp);
extern void _finish_to_record_original_sound(void);
extern void _start_to_record_denoised_original_sound(std::string timestamp);
extern void _finish_to_record_denoised_original_sound(void);

extern int _get_led_based_angle(int mic_angle);
extern int _get_led_based_mic_id(int mic_id);
extern int _set_target_led_on(int led_id);
extern int _get_major_mic_id(void);
extern void _set_major_mic_id(int id);

extern void _set_awake_word(const char *awake_words);

extern std::string _get_denoise_file_path(void);

extern std::string _get_orignal_file_path(void);

namespace ifly_mic_driver
{
IflyMicDriver::IflyMicDriver() : Node("ifly_mic_driver")
{
    // 参数初始化
    initialization_parameters();
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&IflyMicDriver::timer_callback, this));
    feedback_publisher_ = this->create_publisher<std_msgs::msg::String>("file_to_tts", 10);
    pcm_publisher_ = this->create_publisher<std_msgs::msg::String>("pcm_file", 10);
    vad_command_publisher_ = this->create_publisher<std_msgs::msg::String>("vad_command", 10);
    vad_result_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "vad_result", 10, std::bind(&IflyMicDriver::vad_result_subscriber_callback, this, std::placeholders::_1));
    // real time publisher
    rt_vad_publisher_ = this->create_publisher<std_msgs::msg::String>("rt_vad_path", 10);
 
    // 发布唤醒提示语音路径
    feedback_file_publisher(awake_prompt_voice_path_, 3);
    _set_awake_word("你好小巢");
    spdlog::info("ifly_mic_driver node launched");
}

IflyMicDriver::~IflyMicDriver() 
{
    _set_awake_word("你好小小");
}

std::string IflyMicDriver::get_current_time()
{
    std::time_t now = std::time(nullptr);
    std::tm* localTime = std::localtime(&now);

    std::stringstream ss;
    ss << std::setfill('0') << std::setw(4) << localTime->tm_year + 1900 << "_"
       << std::setfill('0') << std::setw(2) << localTime->tm_mon + 1 << "_"
       << std::setfill('0') << std::setw(2) << localTime->tm_mday << "_"
       << std::setfill('0') << std::setw(2) << localTime->tm_hour << "_"
       << std::setfill('0') << std::setw(2) << localTime->tm_min << "_"
       << std::setfill('0') << std::setw(2) << localTime->tm_sec;

    return ss.str();
}

void IflyMicDriver::initialization_parameters()
{
    this->declare_parameter<std::string>("pcm_path", "pcm_path");
    pcm_path_ = this->get_parameter("pcm_path").get_value<std::string>();
    this->declare_parameter<std::string>("awake_prompt_voice_path", "awake_prompt_voice_path");
    awake_prompt_voice_path_ = this->get_parameter("awake_prompt_voice_path").get_value<std::string>();
    this->declare_parameter<std::string>("awake_notice_voice_path", "awake_notice_voice_path");
    awake_notice_voice_path_ = this->get_parameter("awake_notice_voice_path").get_value<std::string>();
    this->declare_parameter<std::string>("sleep_notice_voice_path", "sleep_notice_voice_path");
    sleep_notice_voice_path_ = this->get_parameter("sleep_notice_voice_path").get_value<std::string>();
    this->declare_parameter<std::string>("interactive_mode", "single");
    interactive_mode_ = this->get_parameter("interactive_mode").get_value<std::string>();
    this->declare_parameter<int>("record_duration", 60);
    record_duration_ = this->get_parameter("record_duration").get_value<int>();
    this->declare_parameter<int>("reawake_duration", 300);
    reawake_duration_ = this->get_parameter("reawake_duration").get_value<int>();
    this->declare_parameter<std::string>("pcm_mode", "denoise");
    pcm_mode_ = this->get_parameter("pcm_mode").get_value<std::string>();
    
    // info paramaters
    spdlog::info("pcm_path_: {}", pcm_path_);
    spdlog::info("awake_prompt_voice_path_: {}", awake_prompt_voice_path_);
    spdlog::info("awake_notice_voice_path_: {}", awake_notice_voice_path_);
    spdlog::info("sleep_notice_voice_path_: {}", sleep_notice_voice_path_);
    spdlog::info("interactive_mode_: {}", interactive_mode_);
    spdlog::info("record_duration_: {} s", record_duration_);
    spdlog::info("reawake_duration_: {} s", reawake_duration_);
    spdlog::info("pcm_mode_: {}", pcm_mode_);

    single_count_max_ = (int)(record_duration_ * 1000 / 100);
    sleep_countdown_ = (int)(reawake_duration_ * 1000 / 100);
    spdlog::info("single_count_max_: {}", single_count_max_);
    spdlog::info("sleep_countdown_: {}", sleep_countdown_);
}

void IflyMicDriver::timer_callback()
{
    if (interactive_mode_ == "single") {
        if (_get_awake()) {
            if (!get_record_status()) {
                // 停止当前录音和VAD
                if (get_record_status()) {
                    stop_record();
                    stop_vad();
                }
                start_record(get_current_time());
                // 启动vad检测
                start_vad(_get_denoise_file_path());
            }
            if (timer_cnt_ ++ > single_count_max_) {
                timer_cnt_ = 0;
                stop_record();
                _set_awake(false);
                spdlog::info("single record timeout, 停止录音并进入睡眠模式");
                // 超时停止发布pcm路径
                send_pcm_path(_get_denoise_file_path());
                // 超时停止vad检测
                stop_vad();
            }
        }
    } else if (interactive_mode_ == "multiple") {
        if (_get_awake()) {
            if (!get_record_status()) {
                start_record(get_current_time());
                // 启动vad检测
                // start_vad(_get_denoise_file_path());
                auto msg = std_msgs::msg::String();
                msg.data = _get_denoise_file_path();
                spdlog::info("Publishing message rt_vad: {}", msg.data);
                rt_vad_publisher_->publish(msg);
            }
        }
    } else {
        spdlog::error("Error interactive_mode_ sertting, please set single or multiple");
    }

}

void IflyMicDriver::start_record(const std::string timestamp)
{
    // 发布唤醒提示语音路径
    feedback_file_publisher(awake_notice_voice_path_, 1);
    spdlog::info("current pcm mode: {}, start_record", pcm_mode_);
    if (pcm_mode_ == "denoise") {
        _start_to_record_denoised_sound(timestamp);
    } else if (pcm_mode_ == "original") {
        _start_to_record_original_sound(timestamp);
    } else if (pcm_mode_ == "both") {
        _start_to_record_denoised_original_sound(timestamp);
    }
    set_record_status(true);
}

void IflyMicDriver::stop_record()
{
    spdlog::info("current pcm mode: {}, stop_record", pcm_mode_);
    _set_target_led_on(99);
    if (pcm_mode_ == "denoise") {
        _finish_to_record_denoised_sound();
    } else if (pcm_mode_ == "original") {
        _finish_to_record_original_sound();
    } else if (pcm_mode_ == "both") {
        _finish_to_record_denoised_original_sound();
    }
    set_record_status(false);
}

bool IflyMicDriver::get_record_status()
{
    return record_status_;
}

void IflyMicDriver::set_record_status(bool status)
{
    record_status_ = status;
}

void IflyMicDriver::vad_result_subscriber_callback(const std_msgs::msg::String::SharedPtr msg)
{
    spdlog::info("Received message vad command: {}", msg->data);
    // vad关闭录音
    if (interactive_mode_ == "single") {
        timer_cnt_ = 0;
        stop_record();
        _set_awake(false);
        spdlog::info("vad detection timeout, 停止录音并进入睡眠模式");
        // 超时停止发布pcm路径
        send_pcm_path(_get_denoise_file_path());
    }
}

void IflyMicDriver::send_pcm_path(std::string path)
{
    auto msg = std_msgs::msg::String();
    msg.data = path;
    spdlog::info("发布PCM文件路径: {}", msg.data.c_str());
    pcm_publisher_->publish(msg);
}

void IflyMicDriver::start_vad(const std::string path)
{
    auto msg = std_msgs::msg::String();
    msg.data = path;
    spdlog::info("Publishing message start vad, target: {}", msg.data.c_str());
    vad_command_publisher_->publish(msg);
}

void IflyMicDriver::stop_vad()
{
    auto msg = std_msgs::msg::String();
    msg.data = "stop_vad";
    spdlog::info("Publishing message start vad, target: {}", msg.data.c_str());
    vad_command_publisher_->publish(msg);
}

void IflyMicDriver::feedback_file_publisher(const std::string file_path, const int delay_time)
{
    auto msg = std_msgs::msg::String();
    msg.data = file_path;
    spdlog::info("Publishing message feedback notice: {}", msg.data);
    feedback_publisher_->publish(msg);
    sleep(delay_time);
}


}  // namespace ifly_mic_driver
