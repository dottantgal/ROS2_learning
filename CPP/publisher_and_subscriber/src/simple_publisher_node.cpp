/**
 * @file simple_publisher_node.cpp
 *
 * @brief A basic publisher node
 *        Not recommend this style, composition of multiple nodes in the 
 *        same executable is not possible. Example purpose
 *        It's useful to look how to use WallRate and exceptions
 *
 * @author Antonio Mauro Galiano
 * Contact: https://www.linkedin.com/in/antoniomaurogaliano/
 *
 */


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


using namespace std::chrono_literals;


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("simple_publisher");
  auto pub = node->create_publisher<std_msgs::msg::String>("/my_message", 10);

  std_msgs::msg::String myMessage;
  size_t counter{0};
  rclcpp::WallRate loop_rate(500ms);

  while (rclcpp::ok())
  {
    myMessage.data = "Hello, world! " + std::to_string(counter++);
    RCLCPP_INFO(node->get_logger(), "Publishing: '%s'", myMessage.data.c_str());
    try
    {
      pub->publish(myMessage);
      rclcpp::spin_some(node);
    } catch (const rclcpp::exceptions::RCLError & e)
    {
      RCLCPP_ERROR(node->get_logger(), "Errore type : %s", e.what());
    }
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}