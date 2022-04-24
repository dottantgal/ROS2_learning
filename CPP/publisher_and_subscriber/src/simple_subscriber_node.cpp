/**
 * @file simple_subscriber_node.cpp
 *
 * @brief A basic subscriber node
 *        Not recommend this style, composition of multiple nodes in the 
 *        same executable is not possible. Example purpose
 *
 * @author Antonio Mauro Galiano
 * Contact: https://www.linkedin.com/in/antoniomaurogaliano/
 *
 */

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

std::shared_ptr<rclcpp::Node> node = nullptr;

void TopicCallback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(node->get_logger(), "I heard the message : '%s'", msg->data.c_str());
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  node = std::make_shared<rclcpp::Node>("simple_subscriber");
  auto sub = node->create_subscription<std_msgs::msg::String>("/my_message", 10, TopicCallback);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}