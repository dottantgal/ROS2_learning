/**
 * @file simple_subscriber_class_node.cpp
 *
 * @brief A basic subscriber class node
 *
 * @author Antonio Mauro Galiano
 * Contact: https://www.linkedin.com/in/antoniomaurogaliano/
 *
 */


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


class MySubscriber : public rclcpp::Node
{
private:
  // declaration of the subscriber to a String type message
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  // subscriber callback triggered when a new message is published to the /my_message topic
  void SubscriberCallback(const std_msgs::msg::String::SharedPtr msg);

public:
  MySubscriber(std::string passedNodeName="VOID") : Node(passedNodeName)
  {
    // the subscriber definition which waits messages and triggers the SubscriberCallback
    // when a message is available on the /my_message topic.
    // the binding needs a placeholders to pass the topic message data to the callback
    sub_ = this->create_subscription<std_msgs::msg::String>(
      "/my_message", 10, std::bind(&MySubscriber::SubscriberCallback,
      this, std::placeholders::_1));
  }

};


void MySubscriber::SubscriberCallback(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "I received the message : '%s'", msg->data.c_str());
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MySubscriber>("my_subscriber_node");
  // the subscriber spins waiting for messages doing nothing until a message is available
  rclcpp::spin(node);
  // it's possible to define the two rows above as
  // rclcpp::spin(std::make_shared<MySubscriber>("my_subscriber_node"));
  rclcpp::shutdown();
  return 0;
}
