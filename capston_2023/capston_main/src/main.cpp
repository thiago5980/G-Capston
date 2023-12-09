#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "capston_main/main.hpp"



int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CapstonMain>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}