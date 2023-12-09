#include "rclcpp/rclcpp.hpp"
#include "capston_msgs/srv/waypoints.hpp"

#include <memory>
#include <iostream>
#include <string>
using namespace std;



void add(const std::shared_ptr<capston_msgs::srv::Waypoints::Request> request,
          std::shared_ptr<capston_msgs::srv::Waypoints::Response>     response)
{
  for (auto k : request->points.poses)
  {
    std::cout << k.position.x << " " << k.position.y << std::endl;

  }
  std::cout <<"in"<<std::endl;
  response->getpoint = true;
  std::cout << "out" << response->getpoint << std::endl;
}

int main(int argc, char **argv){
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("test_service");

  rclcpp::Service<capston_msgs::srv::Waypoints>::SharedPtr service =
    node->create_service<capston_msgs::srv::Waypoints>("/ccpp_waypoints", &add);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to calculate two ints.");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}