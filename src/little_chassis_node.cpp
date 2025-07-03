#include "little_chassis/little_chassis_node.hpp"
#include "little_chassis/udp_messages.hpp"
#include <chrono>
#include <cstring>

using namespace std::chrono_literals;
using little_chassis::LittleChassisNode;

/**
 * @brief Constructor for LittleChassisNode.
 * 
 * This function initializes the node, sets up parameters, subscriptions,
 * publishers, and Boost.Asio sockets for communication with the MCU.
 */
LittleChassisNode::LittleChassisNode()
    : Node("little_chassis"),
      cmd_sock_(io_ctx_),
      recv_sock_(io_ctx_)
{
    /* Declare & get parameters */
    declare_parameter("wheel_base", 0.30);
    declare_parameter("wheel_diameter", 0.15);
    declare_parameter("mcu_ip", "192.168.1.100");
    declare_parameter("mcu_cmd_port", 12000);
    declare_parameter("mcu_recv_port", 12000);

    std::string mcu_ip;
    int cmd_port, recv_port;
    get_parameter("wheel_base", wheel_base_);
    get_parameter("wheel_diameter", wheel_diameter_);
    get_parameter("mcu_ip", mcu_ip);
    get_parameter("mcu_cmd_port", cmd_port);
    get_parameter("mcu_recv_port", recv_port);

    /* ROS interfaces */
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "geometry/twist", 10,
        std::bind(&LittleChassisNode::cmdCallback, this, std::placeholders::_1));
    left_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("left_wheel/odom", 10);
    right_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("right_wheel/odom", 10);

    /* Boost.Asio sockets */
    udp::endpoint local_ep(udp::v4(), recv_port);
    recv_sock_.open(udp::v4());
    recv_sock_.bind(local_ep);

    mcu_endpoint_ = udp::endpoint(boost::asio::ip::make_address(mcu_ip), cmd_port);
    cmd_sock_.open(udp::v4());

    startReceive();

    /* Heartbeat timer */
    heartbeat_timer_ = create_wall_timer(100ms, std::bind(&LittleChassisNode::heartBeat, this));

    /* Spin io_context in a background thread */
    running_ = true;
    io_thread_ = std::thread([this]
                             { io_ctx_.run(); });
}

/**
 * @brief Destructor for LittleChassisNode.
 * 
 * This function stops the io_context and joins the io_thread to ensure
 * proper cleanup of resources.
 */
LittleChassisNode::~LittleChassisNode()
{
    running_ = false;
    io_ctx_.stop();
    if (io_thread_.joinable())
        io_thread_.join();
}

/**
 * @brief Callback function for receiving Twist messages.
 * 
 * This function converts the Twist message into a UDP command packet and sends it
 * to the MCU to control the robot's motion.
 * 
 * @param msg The received Twist message containing linear and angular velocities.
 */
void LittleChassisNode::cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    UdpSetRobotMotion_t udpCommandPacket;
    udpCommandPacket.msgType = UDP_MSG_TYPE_SET_ROBOT_MOTION;
    udpCommandPacket.speed = static_cast<float>(msg->linear.x);
    udpCommandPacket.omega = static_cast<float>(msg->angular.z);
    boost::system::error_code ec;
    cmd_sock_.send_to(boost::asio::buffer(&udpCommandPacket, sizeof(udpCommandPacket)),
                      mcu_endpoint_, 0, ec);
    if (ec)
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "send_to error: %s", ec.message().c_str());
}

/**
 * @brief Start asynchronous receive operation on the UDP socket.
 * 
 * This function sets up the socket to listen for incoming packets and
 * binds the handler to process received data.
 */
void LittleChassisNode::startReceive()
{
    recv_sock_.async_receive_from(
        boost::asio::buffer(recv_buf_), sender_endpoint_,
        std::bind(&LittleChassisNode::handleReceive, this, std::placeholders::_1, std::placeholders::_2));
}

/**
 * @brief Handle received UDP packets.
 * 
 * This function processes the received data based on its type and publishes
 * relevant information such as motor speed and system status.
 * 
 * @param ec The error code indicating the result of the receive operation.
 * @param bytes_recvd The number of bytes received in the packet.
 */
void LittleChassisNode::handleReceive(const boost::system::error_code &ec,
                                      std::size_t bytes_recvd)
{
    if (!ec && bytes_recvd > 0)
    {
        const uint32_t type = *reinterpret_cast<const uint32_t *>(recv_buf_.data());
        const auto now = this->now();

        switch (type)
        {
            case UDP_MSG_TYP_HEARTBEAT:
            {
                RCLCPP_DEBUG(get_logger(), "heartbeat received");
                break;
            }
            case UDP_MSG_TYPE_MOTOR_INFO:
            {
                if (bytes_recvd >= sizeof(UdpMotorInfo_t))
                {
                    const auto *m = reinterpret_cast<const UdpMotorInfo_t *>(recv_buf_.data());
                    RCLCPP_DEBUG(get_logger(), "motor info: speed[0]=%.2f, speed[1]=%.2f",
                                 m->speed[0], m->speed[1]);
                    publishWheelOdom(m->position[0] * (wheel_diameter_ * M_PI / 1000000.0),
                                     m->position[1] * (wheel_diameter_ * M_PI / 1000000.0),
                                     now);
                }
                break;
            }
            case UDP_MSG_TYPE_SYSTEM_STATUS:
            {
                if (bytes_recvd >= sizeof(UdpSystemStatus_t))
                {
                    const auto *s = reinterpret_cast<const UdpSystemStatus_t *>(recv_buf_.data());
                    RCLCPP_DEBUG(get_logger(), "system status: voltage=%.2f, current    =%.2f, capacity=%.2f%%",
                                 s->voltage, s->current, s->capacity);
                }
                break;
            }
            default:
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                    "unknown pkt 0x%02x", type);
        }
    }
    else if (ec != boost::asio::error::operation_aborted)
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "recv error: %s", ec.message().c_str());
    }

    if (running_)
        startReceive(); // re‑arm
}

/**
 * @brief Publish wheel odometry messages for left and right wheels.
 * 
 * @param left_dist Distance traveled by the left wheel in metres.
 * @param right_dist Distance traveled by the right wheel in metres.
 * @param stamp Timestamp for the odometry messages.
 */
void LittleChassisNode::publishWheelOdom(double left_dist,
                                         double right_dist,
                                         const rclcpp::Time &stamp)
{
    auto make_msg = [&](double d, const std::string &frame)
    {
        nav_msgs::msg::Odometry msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = frame;
        msg.child_frame_id = "base_link";
        msg.pose.pose.position.x = d;
        return msg;
    };

    left_odom_pub_->publish(make_msg(left_dist, "left_wheel"));
    right_odom_pub_->publish(make_msg(right_dist, "right_wheel"));
}

/**
 * @brief Send a heartbeat packet to the MCU.
 */
void LittleChassisNode::heartBeat()
{
    UdpHeartbeat_t heartbeatPacket;
    heartbeatPacket.msgType = UDP_MSG_TYP_HEARTBEAT;
    boost::system::error_code ec;
    cmd_sock_.send_to(boost::asio::buffer(&heartbeatPacket, sizeof(heartbeatPacket)), mcu_endpoint_, 0, ec);
    if (ec) RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "heartbeat send error: %s", ec.message().c_str());
}
