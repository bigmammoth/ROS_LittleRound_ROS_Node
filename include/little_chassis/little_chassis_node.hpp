#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <boost/asio.hpp>
#include <array>
#include <thread>
#include <atomic>

namespace little_chassis
{
    class LittleChassisNode : public rclcpp::Node
    {
    public:
        LittleChassisNode();
        ~LittleChassisNode();

    private:
        /* ROS --------------------------------------------------*/
        void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
        void publishWheelOdom(double left_dist_m,
                              double right_dist_m,
                              const rclcpp::Time &stamp);
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr left_odom_pub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr right_odom_pub_;
        rclcpp::Logger logger_{get_logger()};
        rclcpp::TimerBase::SharedPtr heartbeat_timer_;

        /* Parameters ------------------------------------------*/
        double wheel_base_{0.0};     //!< metres
        double wheel_diameter_{0.0}; //!< metres

        /* Boost.Asio ------------------------------------------*/
        using udp = boost::asio::ip::udp;
        boost::asio::io_context io_ctx_;
        udp::socket cmd_sock_;
        udp::socket recv_sock_;
        udp::endpoint mcu_endpoint_;
        std::array<uint8_t, 128> recv_buf_{};
        udp::endpoint sender_endpoint_;
        std::thread io_thread_;
        std::atomic<bool> running_{false};

        void startReceive();
        void handleReceive(const boost::system::error_code &ec, std::size_t bytes_recvd);
        void heartBeat();
    };
} // namespace little_chassis
