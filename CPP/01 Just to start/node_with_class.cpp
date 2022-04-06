/**
 * @file node_with_class.cpp
 *
 * @brief A basic ROS2 node implemented with class that spins
 *        and printouts a string waiting for a killing request
 *
 * @author Antonio Mauro Galiano
 * Contact: https://www.linkedin.com/in/antoniomaurogaliano/
 *
 */


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
  auto node = std::make_shared<MyNode>();   // declaration of a MyNode class node
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
