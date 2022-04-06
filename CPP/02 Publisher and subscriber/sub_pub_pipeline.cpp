#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


class SubPubPipeline : public rclcpp::Node
{
private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;    // declaration of the publisher of a String type message
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;     // declaration of the subscriber to a String type message
  std::string topicName_;
  std::string modTopicName_;
  void TopicCallback(const std_msgs::msg::String::SharedPtr msg);

public:
    SubPubPipeline(std::string passedNodeName="/VOID", std::string passedTopicName="/VOID", std::string passedModTopicName="/VOID") 
    : Node(passedNodeName), topicName_(passedTopicName), modTopicName_(passedModTopicName)
    {
        pub_ = this->create_publisher<std_msgs::msg::String>(modTopicName_, 10);    // definition of the publisher to a new topic
        sub_ = this->create_subscription<std_msgs::msg::String>(
            topicName_, 10, std::bind(&SubPubPipeline::TopicCallback, 
            this, std::placeholders::_1));    // definition of the subscriber to the original topic
                                              // placeholder to pass the topic message data to the callback
    }    
};


void SubPubPipeline::TopicCallback(const std_msgs::msg::String::SharedPtr msg)
{
    auto newMsg = std_msgs::msg::String();
    // the line above can be written as std_msgs::msg::String newMsg;
    newMsg.data = "I RESEND " + msg->data;    // new message data is created appending the message got from the topic
    RCLCPP_INFO(this->get_logger(), 
      "Received: %s, Published: %s", msg->data.c_str(), newMsg.data.c_str());   // look that an arrow operator is used
                                                                                // for the msg cause it's a pointer
    pub_->publish(newMsg);    // the modified message is published on a new topic
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SubPubPipeline>("my_pipeline_node", "/my_message", "/my_mod_message");
    rclcpp::spin(node);   // the node spins doing nothing until a message is published on the proper topic 
    rclcpp::shutdown();
    return 0;
}