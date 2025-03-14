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


#ifndef IFLY_MIC_DRIVER__IFLY_MIC_DRIVER_HPP_
#define IFLY_MIC_DRIVER__IFLY_MIC_DRIVER_HPP_

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "spdlog/spdlog.h"
#include "std_msgs/msg/string.hpp"

namespace ifly_mic_driver
{
class IflyMicDriver : public rclcpp::Node
{
public:
    IflyMicDriver();
    ~IflyMicDriver();

private:
    std::string get_current_time();
    void initialization_parameters();
    void timer_callback();
    void start_record(const std::string timestamp);
    void stop_record();
    bool get_record_status();
    void set_record_status(bool status);
    void send_pcm_path(std::string path);
    void vad_result_subscriber_callback(const std_msgs::msg::String::SharedPtr msg);
    void feedback_file_publisher(const std::string file_path, const int delay_time);
    void start_vad(const std::string path);
    void stop_vad();
    
    rclcpp::TimerBase::SharedPtr timer_;
    // feedback
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr feedback_publisher_;
    // pcm文件路径
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pcm_publisher_;
    // vad
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr vad_command_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr vad_result_subscriber_;

    // TODO
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rt_vad_publisher_;
    // parameters
    std::string pcm_path_;
    // 唤醒词提示
    std::string awake_prompt_voice_path_;
    // 唤醒提示
    std::string awake_notice_voice_path_;
    // 睡眠提示
    std::string sleep_notice_voice_path_;
    std::string interactive_mode_;
    int record_duration_;
    int reawake_duration_;
    std::string pcm_mode_;
    bool record_status_{false};
    int single_count_max_{0};
    int timer_cnt_{0};
    // 计时器 进入休眠状态
    int sleep_countdown_;
};
}  // namespace ifly_mic_driver
#endif  // IFLY_MIC_DRIVER__IFLY_MIC_DRIVER_HPP_
