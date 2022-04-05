#include "rclcpp/rclcpp.hpp"

class MyNode : public rclcpp::Node    // class declaration which inherits from rclcpp::Node
{
public:
  MyNode() : Node("ros2_node_class")
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Echo line from a ROS2 class node");   // just a printout
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyNode>();   // created a node of our class
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}