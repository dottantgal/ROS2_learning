#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MySubscriber : public rclcpp::Node
{
private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;    // definition of the subscriber to a String type message
  void SubscriberCallback(const std_msgs::msg::String::SharedPtr msg);    // subscriber callback triggered when a new message
                                                                          // is published on the /my_message topic

public:
  MySubscriber(std::string passedNodeName="VOID") : Node(passedNodeName)
  {
    sub_ = this->create_subscription<std_msgs::msg::String>(
      "/my_message", 10, std::bind(&MySubscriber::SubscriberCallback, 
      this, std::placeholders::_1));    // the subscriber declaration which waits messages and triggers the SubscriberCallback
                                        // when a message is available on the /my_message topic.
                                        // the binding needs a placeholders to pass the topic message data to the callback
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
  rclcpp::spin(node);   // the subscriber spins waiting for messages doing nothing until a message is available
  //    it's possible to define the two rows above as
  //    rclcpp::spin(std::make_shared<MySubscriber>("my_subscriber_node"));
  rclcpp::shutdown();
  return 0;
}