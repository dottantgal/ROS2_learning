/**
 * @file my_first_node.cpp
 *
 * @brief A basic ROS2 node that spins and printouts a string
 *        waiting for a killing request
 *
 * @author Antonio Mauro Galiano
 * Contact: https://www.linkedin.com/in/antoniomaurogaliano/
 *
 */


// import of rclcpp library to retrieve many of the ROS2 core functionalities: nodes, topics, services
#include "rclcpp/rclcpp.hpp"


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);   // initiate ROS2 communications

  // creation of the ROS2 node handled by a shared_ptr
  // to avoid use of new/delete thanks to smart pointers
  // It's equivalent to "std::shared_ptr<rclcpp::Node> node;"
  // but in that case there're two heap allocation to reserve memory space
  // for the control block and the data. With make_shared just one allocation
  auto node = std::make_shared<rclcpp::Node>("ros2_node");

  // use of the the logger method of the class Node to printout info
  RCLCPP_INFO(node->get_logger(), "ROS2 node first test");
  rclcpp::spin(node);   // the node spins until a killing request (Ctrl^C)
  rclcpp::shutdown();   // shutdown of the node
  return 0;   // return to the operating system
}
