#include "rclcpp/rclcpp.hpp"
#include "rm_set_posture/set_posture_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SetPostureNode>());
  rclcpp::shutdown();
  return 0;
}
