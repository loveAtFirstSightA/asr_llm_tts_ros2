#ifndef INTERFACE_HPP
#define INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <boost/asio.hpp>
#include <memory>
#include <thread>
#include <chrono>
#include <spdlog/spdlog.h>

namespace interface
{
    class Interface : public rclcpp::Node
    {
    public:
        Interface();
        ~Interface();

    private:
        void init_parameters();
        void subscriber_callback(const std_msgs::msg::String::SharedPtr msg);
        void send_message_to_server(const std::string &message);
        void connect_with_retry(boost::asio::ip::tcp::socket &socket, const boost::asio::ip::tcp::endpoint &endpoint);
        void reconnect();
        void check_connection_status();

        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
        std::unique_ptr<boost::asio::io_context> io_context_;
        std::unique_ptr<boost::asio::ip::tcp::socket> socket_;
        boost::asio::ip::tcp::endpoint server_endpoint_;
        std::unique_ptr<std::thread> io_thread_;
        rclcpp::TimerBase::SharedPtr connection_check_timer_;

        std::string server_address_;
        int port_;
    };
}  // namespace interface

#endif  // INTERFACE_HPP
