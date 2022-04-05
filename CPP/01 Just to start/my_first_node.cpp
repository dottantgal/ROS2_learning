#include "rclcpp/rclcpp.hpp"    // import of rclcpp library to retrieve many of the ROS2 core functionalities: nodes, topics, services

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);   // initiate ROS2 communications

  auto node = std::make_shared<rclcpp::Node>("ros2_node");    // creation of the ROS2 node handled by a shared_ptr
                                                              // to avoid use of new/delete thanks to smart pointers
                                                              // auto is equivalent to std::shared_ptr<rclcpp::Node>

  RCLCPP_INFO(node->get_logger(), "ROS2 node first test");    //  use of the the logger method of the class Node to printout info
  rclcpp::spin(node);   // the node spins until a killing request (Ctrl^C)
  rclcpp::shutdown();   // shutdown of the node
  return 0;   // return to the operating system
}