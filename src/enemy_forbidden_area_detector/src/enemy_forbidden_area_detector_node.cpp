#include <rclcpp/rclcpp.hpp>
#include "enemy_forbidden_area_detector/enemy_forbidden_area_detector.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<
      enemy_forbidden_area_detector::EnemyForbiddenAreaDetector>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
