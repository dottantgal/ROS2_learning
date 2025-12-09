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
#include <csignal>

// global declaration of the node to be called inside the timer
// callback. It is not a good practice, it's just an example
std::shared_ptr<rclcpp::Node> node = nullptr;
rclcpp::TimerBase::SharedPtr timer = nullptr;

void TimerCallback()
{
  if (node && rclcpp::ok()) {
    RCLCPP_INFO(node->get_logger(), "Echo line from a ROS2 timer callback");
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  node = std::make_shared<rclcpp::Node>("ros2_node_class_timer");
  timer = node->create_wall_timer(std::chrono::milliseconds(200),
      TimerCallback);       // timer type is rclcpp::TimerBase::SharedPtr
  
  try {
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Exception in spin: %s", e.what());
  }
  
  // Graceful shutdown: reset timer before node destruction
  if (timer) {
    timer->cancel();
    timer.reset();
  }
  
  // Reset node before shutdown
  node.reset();
  
  rclcpp::shutdown();
  return 0;
}
