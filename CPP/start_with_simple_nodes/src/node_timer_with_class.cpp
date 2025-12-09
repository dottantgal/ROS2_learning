/**
 * @file node_timer_with_class.cpp
 *
 * @brief A basic ROS2 node implemented with class that spins
 *        and printouts a string based on a timer
 *
 * @author Antonio Mauro Galiano
 * Contact: https://www.linkedin.com/in/antoniomaurogaliano/
 *
 */


#include "rclcpp/rclcpp.hpp"


class MyNode : public rclcpp::Node
{
private:
  rclcpp::TimerBase::SharedPtr timer_;    // define a ROS2 timer object contained inside a shared ptr
  void TimerCallback();   // callback for the timer, what it will do when called

public:
  MyNode() : Node("ros2_node_class_timer")
  {
    // the timer is initialized with an inherit function from rclcpp::Node
    // passing two arguments: the frequency to call the callback and
    // the binded callback itself (binded to be able to pass the class method)
    // which is triggered when the node starts spinning
    timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&MyNode::TimerCallback,this));
  }
};


void MyNode::TimerCallback()    // simple callback triggered by the timer to printout info
{
  if (rclcpp::ok()) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Echo line from a ROS2 class node with timer");
  }
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyNode>();
  
  try {
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Exception in spin: %s", e.what());
  }
  
  // Graceful shutdown: node will clean up timer automatically via RAII
  node.reset();
  rclcpp::shutdown();
  return 0;
}
