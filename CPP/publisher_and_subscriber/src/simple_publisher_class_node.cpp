/**
 * @file simple_publisher_class_node.cpp
 *
 * @brief A basic publisher class node
 *
 * @author Antonio Mauro Galiano
 * Contact: https://www.linkedin.com/in/antoniomaurogaliano/
 *
 */
 

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MyPublisher : public rclcpp::Node
{
private:
  size_t counter_;    // the published messages counter
  rclcpp::TimerBase::SharedPtr timer_;

  // declaration of the publisher object throught a shared ptr
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;   
  void TimerCallback();

public:
  MyPublisher(std::string passedNodeName="VOID", size_t passedCounter=0)
    : Node(passedNodeName), counter_(passedCounter)
  {
    // publisher definition. It publishes a message of String type
    // to "/my_message" topic with a queue of 10 positions
    pub_ = this->create_publisher<std_msgs::msg::String>("/my_message", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&MyPublisher::TimerCallback, this));
  }
};

void MyPublisher::TimerCallback()
{
  std_msgs::msg::String message;    // message to publish of String type
  // the line above can be written as "auto message = std_msgs::msg::String();"
  message.data = "Hello, world! " + std::to_string(counter_++);   // the data of the message
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());    // printout of the message
  pub_->publish(message);   // publish method
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  // node declaration of class MyPublisher passing the name of the node and the counter init value
  auto node = std::make_shared<MyPublisher>("my_publisher_node", 0);
  rclcpp::spin(node);
  // it's possibile to define the two lines above using just one
  // rclcpp::spin(std::make_shared<MyPublisher>())

  rclcpp::shutdown();
  return 0;
}
