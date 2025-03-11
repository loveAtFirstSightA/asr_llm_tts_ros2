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
    void initialization_parameters();
    void vad_subscriber_callback(const std_msgs::msg::String::SharedPtr msg);
    void timer_callback();
    bool get_record_status();
    void set_record_status(bool status);
    void auto_start_record();
    void auto_stop_record();
    void publish_pcm_file_path(std::string path);
    void publish_awake_notice_voice(std::string path);
    void publish_vad_command(std::string command);
    void start_vad();
    void stop_vad();
    void vad_stop_record();
    
    rclcpp::TimerBase::SharedPtr timer_;
    // pcm文件路径
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pcm_publisher_;
    // 语音路径
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr awake_file_publisher_;
    // vad
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr vad_command_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr vad_result_subscriber_;
    // parameters
    std::string pcm_path_;
    std::string awake_prompt_voice_path_;
    std::string awake_notice_voice_path_;
    int record_duration_;

    bool record_status_{false};
    int timer_count_max_{0};
    int timer_cnt_{0};
};
}  // namespace ifly_mic_driver
#endif  // IFLY_MIC_DRIVER__IFLY_MIC_DRIVER_HPP_
