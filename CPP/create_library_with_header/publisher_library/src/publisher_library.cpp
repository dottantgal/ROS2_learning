/**
 * @file publisher_library.cpp
 *
 * @brief A basic publisher library made with header file
 *
 * @author Antonio Mauro Galiano
 * Contact: https://www.linkedin.com/in/antoniomaurogaliano/
 *
 */

#include "publisher_library/publisher_library.h"

PublisherLibrary::PublisherLibrary() : Node("ros2_publisher_library"), counter_(0)
{
  pub_ = this->create_publisher<std_msgs::msg::String>("/my_published_msg", 10);
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500), std::bind(&PublisherLibrary::TimerCallback, this));
}

void PublisherLibrary::TimerCallback()
{
  std_msgs::msg::String message;
  message.data = "HELLO WORLD number " + std::to_string(counter_++);
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  pub_->publish(message);
}

PublisherLibrary::~PublisherLibrary() {}