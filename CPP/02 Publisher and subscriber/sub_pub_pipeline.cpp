/**
 * @file sub_pub_pipeline.cpp
 *
 * @brief Simple node that creates a pipeline subscribing to the /my_message topic published
 *        by simple_publisher_class_node.cpp node, it modifies the message data and publishes
 *        the modified message to the /my_mod_message topic
 *
 * @author Antonio Mauro Galiano
 * Contact: https://www.linkedin.com/in/antoniomaurogaliano/
 *
 */


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


class SubPubPipeline : public rclcpp::Node
{
private:
  // declaration of the publisher to a String type message
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  // declaration of the subscriber to a String type message
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  std::string topicName_;
  std::string modTopicName_;
  void TopicCallback(const std_msgs::msg::String::SharedPtr msg);

public:
  SubPubPipeline(std::string passedNodeName="/VOID", std::string passedTopicName="/VOID", std::string passedModTopicName="/VOID")
    : Node(passedNodeName), topicName_(passedTopicName), modTopicName_(passedModTopicName)
  {
    // definition of the publisher to a new topic
    pub_ = this->create_publisher<std_msgs::msg::String>(modTopicName_, 10);
    
    // definition of the subscriber to the original topic
    // placeholder to pass the topic message data to the callback
    sub_ = this->create_subscription<std_msgs::msg::String>(
        topicName_, 10, std::bind(&SubPubPipeline::TopicCallback,
        this, std::placeholders::_1));                                
  }
};


void SubPubPipeline::TopicCallback(const std_msgs::msg::String::SharedPtr msg)
{
  auto newMsg = std_msgs::msg::String();
  // the line above can be written as std_msgs::msg::String newMsg;
  
  // new message data is created appending the message got from the topic
  newMsg.data = "I RESEND " + msg->data;
  
  // look that an arrow operator is used for the msg cause it's a pointer
  RCLCPP_INFO(this->get_logger(),
              "Received: %s, Published: %s", msg->data.c_str(), newMsg.data.c_str());
  pub_->publish(newMsg);    // the modified message is published on a new topic
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SubPubPipeline>("my_pipeline_node", "/my_message", "/my_mod_message");
  // the node spins doing nothing until a message is published on the proper topic
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
