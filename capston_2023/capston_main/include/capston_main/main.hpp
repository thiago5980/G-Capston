#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "capston_msgs/srv/robotstates.hpp"
#include "capston_msgs/msg/motorcapston.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;

class CapstonMain : public rclcpp::Node
{
public:
  CapstonMain() : Node("capston_main_node")
    {
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
        start_publisher_ = this->create_publisher<capston_msgs::msg::Motorcapston>("start_robot", rclcpp::SensorDataQoS());
        sequence_end_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
          "end_sequence",
          qos_profile,
          std::bind(&CapstonMain::subscribe_end_sequence_message, this, _1));
        plancaller_ = this->create_client<capston_msgs::srv::Robotstates>("calculate_plan");

        timer_ = this->create_wall_timer( // start clean that end_flag_ is false, if end_flag is true end clean 
          1s,
          [this]() -> void
            {
                auto msg = capston_msgs::msg::Motorcapston();
                if (this->start_flag_ && !this->end_flag_)
                {
                    std::cout << "all motor on" << std::endl;
                    msg.cleaner = true;
                    msg.brush = true;
                    msg.water = false;
                }
                else
                {
                    msg.cleaner = false;
                    msg.brush = false;
                    msg.water = false;
                }
                this->start_publisher_->publish(msg);
            }
        );

        this->declare_parameter("origin_x", 0.0);
        this->declare_parameter("origin_y", 0.0);
        this->declare_parameter("start_x", 0.0);
        this->declare_parameter("start_y", 0.0);
        this->declare_parameter("robot_radius", 0);
        this->declare_parameter("sweep_step", 0);
        this->declare_parameter("map_height", 0);
        this->declare_parameter("map_location", "/home");

        this->get_parameter("origin_x", this->origin_.position.x);
        this->get_parameter("origin_y", this->origin_.position.y);
        this->get_parameter("start_x", this->start_position_.position.x);
        this->get_parameter("start_y", this->start_position_.position.y);
        this->get_parameter("robot_radius", this->robot_radius_);
        this->get_parameter("sweep_step", this->sweep_step_);
        this->get_parameter("map_height", this->map_height);
        this->get_parameter("map_location", this->map_location);

        std::cout << "origin_x: " << this->origin_.position.x << std::endl;
        std::cout << "origin_y: " << this->origin_.position.y << std::endl;
        std::cout << "start_x: " << this->start_position_.position.x << std::endl;
        std::cout << "start_y: " << this->start_position_.position.y << std::endl;
        std::cout << "robot_radius: " << this->robot_radius_ << std::endl;
        std::cout << "sweep_step: " << this->sweep_step_ << std::endl;
        std::cout << "map_height: " << this->map_height << std::endl;
        std::cout << "map_location: " << this->map_location << std::endl;

        while (!plancaller_->wait_for_service(1s)) {
            if (!rclcpp::ok()) 
            {
              RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
              return;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }
        this->send_robotstates();
    }



private:
    void subscribe_end_sequence_message(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data) 
            this->end_flag_ = true;
        else
            this->end_flag_ = false;
    }

    void send_robotstates()
    {
        auto request = std::make_shared<capston_msgs::srv::Robotstates::Request>();
        request->origin.x = this->origin_.position.x;
        request->origin.y = this->origin_.position.y;
        request->start_position.x = this->start_position_.position.x;
        request->start_position.y = this->start_position_.position.y;
        request->robot_radius = this->robot_radius_;
        request->sweep_step = this->sweep_step_;
        request->map_height = this->map_height;
        request->map_location = this->map_location;

        using ServiceResponseFuture = rclcpp::Client<capston_msgs::srv::Robotstates>::SharedFuture;
        auto response_received_callback = [this](ServiceResponseFuture future){
            auto response = future.get();
            if (response->end_cal)
            {
                std::cout << "end_cal" << std::endl;
                this->start_flag_ = true;
            }
            else
                this->start_flag_ = false;
            return;
        };
        auto future_result = plancaller_->async_send_request(request, response_received_callback);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<capston_msgs::msg::Motorcapston>::SharedPtr start_publisher_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sequence_end_subscription_;
    rclcpp::Client<capston_msgs::srv::Robotstates>::SharedPtr plancaller_;

    bool end_flag_ = false;
    bool start_flag_ = false;

    int robot_radius_ = 0;
    int sweep_step_ = 0;
    int map_height = 0;
    
    std::string map_location;

    geometry_msgs::msg::Pose origin_ = geometry_msgs::msg::Pose();
    geometry_msgs::msg::Pose start_position_ = geometry_msgs::msg::Pose();


};
