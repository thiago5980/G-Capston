#include <iostream>
#include <vector>
#include "capston_plan/capstonplanning.hpp"
#include "rclcpp/rclcpp.hpp"
#include "capston_msgs/srv/waypoints.hpp"
#include <chrono>
#include <cstdlib>
#include <memory>
using namespace std::chrono_literals;


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GetWaypoints>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}