/**
 * @file node_timer_without_class.cpp
 *
 * @brief A basic ROS2 node that spins and printouts a string
 *        based on a timer. Not good practice, just for instance
 *
 * @author Antonio Mauro Galiano
 * Contact: https://www.linkedin.com/in/antoniomaurogaliano/
 *
 */


#include "rclcpp/rclcpp.hpp"

// global declaration of the node to be called inside the timer
// callback. It is not a good practice, it's just an example
std::shared_ptr<rclcpp::Node> node = nullptr;


void TimerCallback()
{
  RCLCPP_INFO(node->get_logger(), "Echo line from a ROS2 timer callback");
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  node = std::make_shared<rclcpp::Node>("ros2_node_class_timer");
  auto timer = node->create_wall_timer(std::chrono::milliseconds(200),
      TimerCallback);       // timer type is rclcpp::TimerBase::SharedPtr
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
