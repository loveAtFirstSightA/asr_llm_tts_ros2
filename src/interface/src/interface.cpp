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

#include "interface/interface.hpp"
#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>
#include <boost/asio.hpp>

namespace interface
{
Interface::Interface() : Node("interface")
{
    init_parameters();

    // 创建 io_context 和 socket
    io_context_ = std::make_unique<boost::asio::io_context>();
    socket_ = std::make_unique<boost::asio::ip::tcp::socket>(*io_context_);

    // 创建服务器端点
    boost::asio::ip::tcp::resolver resolver(*io_context_);
    server_endpoint_ = *resolver.resolve(server_address_, std::to_string(port_)).begin();

    // 尝试连接服务器，带重试机制
    connect_with_retry(*socket_, server_endpoint_);

    // 创建定时器用于定期检测连接状态
    connection_check_timer_ = this->create_wall_timer(
        std::chrono::seconds(5), std::bind(&Interface::check_connection_status, this));

    // 创建订阅者
    subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "interface_msg", 10, std::bind(&Interface::subscriber_callback, this, std::placeholders::_1));

    // 启动 io_context 线程
    io_thread_ = std::make_unique<std::thread>([this]() {
        try {
            io_context_->run();
        } catch (const std::exception &e) {
            spdlog::error("IO 线程异常: {}", e.what());
        }
    });
}

Interface::~Interface()
{
    if (socket_ && socket_->is_open()) {
        socket_->close();
        spdlog::info("Socket 连接已关闭");
    }
    if (io_thread_ && io_thread_->joinable()) {
        io_thread_->join();
        spdlog::info("IO 线程已退出");
    }
}

void Interface::init_parameters()
{
    this->declare_parameter<int>("port", 3377);
    port_ = this->get_parameter("port").get_value<int>();
    spdlog::info("Socket 端口: {}", port_);

    this->declare_parameter<std::string>("server_address", "127.0.0.1");
    server_address_ = this->get_parameter("server_address").get_value<std::string>();
    spdlog::info("Socket 服务器地址: {}", server_address_);
}

void Interface::subscriber_callback(const std_msgs::msg::String::SharedPtr msg)
{
    spdlog::info("收到 ROS 2 消息: '{}'", msg->data.c_str());

    // 发送消息到服务器
    send_message_to_server(msg->data);
}

void Interface::send_message_to_server(const std::string &message)
{
    // 检查连接是否有效
    if (!socket_->is_open()) {
        spdlog::warn("目标端口 {} 连接断开，正在重连...", port_);
        reconnect();
    }

    // 发送数据
    boost::asio::async_write(*socket_, boost::asio::buffer(message + "\n"),
        [this, message](const boost::system::error_code &ec, std::size_t bytes_transferred) {
            if (ec) {
                spdlog::error("发送消息失败: {}", ec.message());
                reconnect();  // 如果发送失败，尝试重新连接
            } else {
                spdlog::info("已发送 {} 字节到服务器", bytes_transferred);
            }
        });
}

void Interface::connect_with_retry(boost::asio::ip::tcp::socket &socket, const boost::asio::ip::tcp::endpoint &endpoint)
{
    boost::system::error_code ec;
    while (true) {
        socket.connect(endpoint, ec);
        if (!ec) {
            spdlog::info("目标端口 {} 连接成功!", port_);
            break;
        } else {
            // TODO debug shield
            // spdlog::warn("目标端口 {} 连接失败: {}, 正在重试...", port_, ec.message());
            std::this_thread::sleep_for(std::chrono::seconds(1));  // 等待 1 秒后重试
        }
    }
}

void Interface::reconnect()
{
    // 重试连接
    spdlog::info("尝试重新连接服务器... 目标端口 {}", port_);
    socket_->close();  // 关闭当前 socket
    socket_ = std::make_unique<boost::asio::ip::tcp::socket>(*io_context_);  // 创建新的 socket
    connect_with_retry(*socket_, server_endpoint_);  // 尝试重新连接
}

void Interface::check_connection_status()
{
    // 定期检查连接状态，检测连接是否有效
    if (!socket_->is_open()) {
        spdlog::warn("目标端口 {} 连接已断开，尝试重新连接...", port_);
        reconnect();
    } else {
        // 检查连接是否成功，发送一个小的请求以确认连接是否活跃
        boost::system::error_code ec;
        char data[1] = {'\0'};
        boost::asio::write(*socket_, boost::asio::buffer(data, 1), ec);
        
        if (ec) {
            spdlog::warn("目标端口 {} 连接检查失败: {}",port_, ec.message());
            reconnect();
        } else {
            spdlog::info("目标端口 {} 连接状态良好", port_);
        }
    }
}
} // namespace interface
