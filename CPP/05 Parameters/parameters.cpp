/**
 * @file set_parameters.cpp
 *
 * @brief A node to declare and get parameters
 *        Here the parameters are the message data for two publisher
 *        It's possible to change them at run time using the commad line
 *        "ros2 param set /set_parameter_node vehicle_speed 100"
 *        "ros2 param set /set_parameter_node vehicle_type car"
 *        or using a launch file
 *
 * @author Antonio Mauro Galiano
 * Contact: https://www.linkedin.com/in/antoniomaurogaliano/
 *
 */

#include <chrono>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int16.hpp"

using namespace std::chrono_literals;

class MySetParameterClass: public rclcpp::Node
{
private:
  int velocityParam_;
  std::string typeVehicleParam_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pubString_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pubInt_;
  void TimerCallback();

public:
  MySetParameterClass()
    : Node("set_parameter_node")
  {
    // here are declared the parameters and their default values
    this->declare_parameter<std::string>("vehicle_type", "bike");
    this->declare_parameter<int>("vehicle_speed", 10);
    pubString_ = this->create_publisher<std_msgs::msg::String>("/vehicle_type", 10);
    pubInt_ = this->create_publisher<std_msgs::msg::Int16>("/vehicle_speed", 10);
    timer_ = this->create_wall_timer(
      1000ms, std::bind(&MySetParameterClass::TimerCallback, this));
  }
};


void MySetParameterClass::TimerCallback()
{
  // here the params get their value from outside
  // such as a set command or a launch file
  this->get_parameter("vehicle_type", typeVehicleParam_);
  this->get_parameter("vehicle_speed", velocityParam_);
  std_msgs::msg::String messageString;
  messageString.data=typeVehicleParam_;
  std_msgs::msg::Int16 messageInt;
  messageInt.data=velocityParam_;
  RCLCPP_INFO(this->get_logger(), "Publishing two messages -> vehicle type %s\tVehicle speed %ld",
    typeVehicleParam_.c_str(), velocityParam_);
  pubInt_->publish(messageInt);
  pubString_->publish(messageString);
}


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MySetParameterClass>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}