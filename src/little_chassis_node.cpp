#include "little_chassis/little_chassis_node.hpp"
#include <chrono>
#include <cstring>

using namespace std::chrono_literals;
using little_chassis::LittleChassisNode;

/* -------------------------------- Constructors -------------------------------- */
LittleChassisNode::LittleChassisNode()
: Node("little_chassis"),
  cmd_sock_(io_ctx_),
  recv_sock_(io_ctx_)
{
  /* Declare & get parameters */
  declare_parameter("wheel_base",     0.30);
  declare_parameter("wheel_diameter", 0.15);
  declare_parameter("mcu_ip",         "192.168.1.100");
  declare_parameter("mcu_cmd_port",   12000);
  declare_parameter("mcu_recv_port",  12001);

  std::string mcu_ip;
  int cmd_port, recv_port;
  get_parameter("wheel_base",      wheel_base_);
  get_parameter("wheel_diameter",  wheel_diameter_);
  get_parameter("mcu_ip",          mcu_ip);
  get_parameter("mcu_cmd_port",    cmd_port);
  get_parameter("mcu_recv_port",   recv_port);

  /* ROS interfaces */
  cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
               "geometry/twist", 10,
               std::bind(&LittleChassisNode::cmdCallback, this, std::placeholders::_1));
  left_odom_pub_  = create_publisher<nav_msgs::msg::Odometry>("left_wheel/odom", 10);
  right_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("right_wheel/odom", 10);

  /* Boost.Asio sockets */
  udp::endpoint local_ep(udp::v4(), recv_port);
  recv_sock_.open(udp::v4());
  recv_sock_.bind(local_ep);

  mcu_endpoint_ = udp::endpoint(boost::asio::ip::make_address(mcu_ip), cmd_port);
  cmd_sock_.open(udp::v4());

  startReceive();

  /* Spin io_context in a background thread */
  running_ = true;
  io_thread_ = std::thread([this]{ io_ctx_.run(); });
}

LittleChassisNode::~LittleChassisNode()
{
  running_ = false;
  io_ctx_.stop();
  if (io_thread_.joinable()) io_thread_.join();
}

/* -------------------------------- ROS Callback -------------------------------- */
// Simple command packet: {float32 linear_mps, float32 angular_rps}
struct CmdPacket
{
  float linear;
  float angular;
};

void LittleChassisNode::cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  CmdPacket pkt{static_cast<float>(msg->linear.x), static_cast<float>(msg->angular.z)};
  boost::system::error_code ec;
  cmd_sock_.send_to(boost::asio::buffer(&pkt, sizeof(pkt)), mcu_endpoint_, 0, ec);
  if (ec)
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "send_to error: %s", ec.message().c_str());
}

/* ----------------------------- Asynchronous Receive --------------------------- */
struct OdomPacket  { uint8_t type; float left_dist;  float right_dist;  };
struct WheelSpdPkt { uint8_t type; float left_rps;   float right_rps;   };
struct StatusPkt   { uint8_t type; uint8_t motor_enable; float battery_voltage; float motor_current; };

void LittleChassisNode::startReceive()
{
  recv_sock_.async_receive_from(
    boost::asio::buffer(recv_buf_), sender_endpoint_,
    std::bind(&LittleChassisNode::handleReceive, this, std::placeholders::_1, std::placeholders::_2));
}

void LittleChassisNode::handleReceive(const boost::system::error_code & ec,
                                      std::size_t bytes_recvd)
{
  if (!ec && bytes_recvd > 0)
  {
    const uint8_t type = recv_buf_[0];
    const auto now = this->now();

    switch (type)
    {
      case 0x00: // odometry
      {
        if (bytes_recvd >= sizeof(OdomPacket))
        {
          const auto * p = reinterpret_cast<const OdomPacket*>(recv_buf_.data());
          publishWheelOdom(p->left_dist, p->right_dist, now);
        }
        break;
      }
      case 0x02: // status
      {
        if (bytes_recvd >= sizeof(StatusPkt))
        {
          const auto * s = reinterpret_cast<const StatusPkt*>(recv_buf_.data());
          RCLCPP_DEBUG(get_logger(), "status enabled=%d, V=%.2f, I=%.2f",
                       s->motor_enable, s->battery_voltage, s->motor_current);
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

  if (running_) startReceive(); // re‑arm
}

/* -------------------------------- Helper -------------------------------- */
void LittleChassisNode::publishWheelOdom(double left_dist,
                                         double right_dist,
                                         const rclcpp::Time & stamp)
{
  auto make_msg = [&](double d, const std::string & frame) {
    nav_msgs::msg::Odometry msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame;
    msg.child_frame_id = "base_link";
    msg.pose.pose.position.x = d;
    return msg;
  };

  left_odom_pub_->publish(make_msg(left_dist,  "left_wheel"));
  right_odom_pub_->publish(make_msg(right_dist, "right_wheel"));
}
