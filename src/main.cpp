#include "little_chassis/little_chassis_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<little_chassis::LittleChassisNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
