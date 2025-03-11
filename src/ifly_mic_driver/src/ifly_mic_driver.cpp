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

extern int _get_boot(void);
extern int _get_awake(void);
extern void _set_awake(int status);
extern void _start_to_record_denoised_sound(void);
extern void _finish_to_record_denoised_sound(void);
extern void _start_to_record_original_sound(void);
extern void _finish_to_record_original_sound(void);
extern void _start_to_record_denoised_original_sound(void);
extern void _finish_to_record_denoised_original_sound(void);

extern int _get_led_based_angle(int mic_angle);
extern int _get_led_based_mic_id(int mic_id);
extern int _set_target_led_on(int led_id);
extern int _get_major_mic_id(void);
extern void _set_major_mic_id(int id);

extern void _set_awake_word(const char *awake_words);

namespace ifly_mic_driver
{
IflyMicDriver::IflyMicDriver() : Node("ifly_mic_driver")
{
    initialization_parameters();
    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&IflyMicDriver::timer_callback, this));
    pcm_publisher_ = this->create_publisher<std_msgs::msg::String>("pcm_file", 10);
    awake_file_publisher_ = this->create_publisher<std_msgs::msg::String>("file_to_tts", 10);
    vad_command_publisher_ = this->create_publisher<std_msgs::msg::String>("vad_command", 10);
    vad_result_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "vad_result", 10, std::bind(&IflyMicDriver::vad_subscriber_callback, this, std::placeholders::_1));

    // 发布唤醒提示语音路径
    auto msg = std_msgs::msg::String();
    msg.data = awake_prompt_voice_path_;
    awake_file_publisher_->publish(msg);
    sleep(3);   //  等待播放结束
    _set_awake_word("你好小巢");
    spdlog::info("ifly_mic_driver node launched");
}

IflyMicDriver::~IflyMicDriver() 
{
    _set_awake_word("你好小小");
}

void IflyMicDriver::initialization_parameters()
{
    this->declare_parameter<std::string>("pcm_path", "pcm_path");
    pcm_path_ = this->get_parameter("pcm_path").get_value<std::string>();
    this->declare_parameter<std::string>("awake_prompt_voice_path", "awake_prompt_voice_path");
    awake_prompt_voice_path_ = this->get_parameter("awake_prompt_voice_path").get_value<std::string>();
    this->declare_parameter<std::string>("awake_notice_voice_path", "awake_notice_voice_path");
    awake_notice_voice_path_ = this->get_parameter("awake_notice_voice_path").get_value<std::string>();
    this->declare_parameter<int>("record_duration", 60);
    record_duration_ = this->get_parameter("record_duration").get_value<int>();
    // info paramaters
    spdlog::info("pcm_path_: {}", pcm_path_);
    spdlog::info("awake_prompt_voice_path_: {}", awake_prompt_voice_path_);
    spdlog::info("awake_notice_voice_path_: {}", awake_notice_voice_path_);
    spdlog::info("record_duration_: {}", record_duration_);
    spdlog::info("可允许最大录音时长： {} s", record_duration_);

    timer_count_max_ = (int)(record_duration_ * 1000 / 500);
    // spdlog::info("timer_count_max_: {}", timer_count_max_);
}

void IflyMicDriver::timer_callback()
{
    if (_get_awake()) {
        // spdlog::info("检测到唤醒， 开始录音");
        if (!get_record_status()) {
            auto_start_record();
        }
        if (timer_cnt_++ > timer_count_max_) {
            timer_cnt_ = 0;
            // 超时自动停止
            auto_stop_record();
        }
    }
}

void IflyMicDriver::vad_subscriber_callback(const std_msgs::msg::String::SharedPtr msg)
{
    spdlog::info("Received message vad command: {}", msg->data);
    vad_stop_record();
}

bool IflyMicDriver::get_record_status()
{
    return record_status_;
}

void IflyMicDriver::set_record_status(bool status)
{
    record_status_ = status;
}

void IflyMicDriver::auto_start_record()
{
    publish_awake_notice_voice(awake_notice_voice_path_);
    // 延时1s等待唤醒语音结束
    sleep(1);
    // _start_to_record_denoised_original_sound();
    _start_to_record_denoised_sound();
    set_record_status(true);
    // 开始vad检测
    publish_vad_command("start");
}

void IflyMicDriver::auto_stop_record()
{
    // 关灯
    _set_target_led_on(99);
    _set_awake(0);
    spdlog::info("检测计时结束，自动结束录音");
    // _finish_to_record_denoised_original_sound();
    _finish_to_record_denoised_sound();
    // 延时 等待硬件结束
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    publish_pcm_file_path(pcm_path_);
    set_record_status(false);
    publish_vad_command("stop");
}

void IflyMicDriver::publish_pcm_file_path(std::string path)
{
    auto msg = std_msgs::msg::String();
    msg.data = path;
    spdlog::info("发布PCM文件路径: {}", msg.data.c_str());
    pcm_publisher_->publish(msg);
}

void IflyMicDriver::publish_awake_notice_voice(std::string path)
{
    // 发布唤醒语音路径
    auto msg = std_msgs::msg::String();
    msg.data = awake_notice_voice_path_;
    spdlog::info("发布唤醒提示语音文件路径: {}", msg.data.c_str());
    awake_file_publisher_->publish(msg);
}

void IflyMicDriver::vad_stop_record()
{
    // 关灯
    _set_target_led_on(99);
    _set_awake(0);
    timer_cnt_ = 0;
    spdlog::info("VAD检测默音超时，自动结束录音");
    // _finish_to_record_denoised_original_sound();
    _finish_to_record_denoised_sound();
    // 延时 等待硬件结束
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    set_record_status(false);
    publish_pcm_file_path(pcm_path_);
    
    // publish_vad_command("stop");
}

void IflyMicDriver::start_vad()
{
    auto msg = std_msgs::msg::String();
    msg.data = pcm_path_;
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

void IflyMicDriver::publish_vad_command(std::string command)
{
    if (command == "start") {
        start_vad();
    }
    if (command == "stop") {
        stop_vad();
    }
}


}  // namespace ifly_mic_driver
