#include "rclcpp/rclcpp.hpp"

std::shared_ptr<rclcpp::Node> node = nullptr;       // global declaration of the node to be called inside the timer
                                                    // callback it is not a good practice, its just an example

void TimerCallback()
{
  RCLCPP_INFO(node->get_logger(), "Echo line from a ROS2 timer callback");
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  node = std::make_shared<rclcpp::Node>("ros2_node_class_timer");
  auto timer = node->create_wall_timer(
      std::chrono::milliseconds(200),
      TimerCallback);       // timer is type rclcpp::TimerBase::SharedPtr
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}