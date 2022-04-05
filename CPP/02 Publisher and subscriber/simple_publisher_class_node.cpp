#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MyPublisher : public rclcpp::Node
{
private:
  size_t counter_;    // the published messages counter
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;   // declaration of the publisher object
                                                              // throught a shared ptr
  void TimerCallback();

public:
  MyPublisher(std::string passedNodeName="VOID", size_t passedCounter=0) 
    : Node(passedNodeName), counter_(passedCounter)
  {
    pub_ = this->create_publisher<std_msgs::msg::String>("/my_message", 10);    // publisher definition
                                                                                // it publishes a message of 
                                                                                // string type
                                                                                // "my_message" topic
                                                                                // queue of 10 positions
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&MyPublisher::TimerCallback, this));
  }
};

void MyPublisher::TimerCallback()
{
  std_msgs::msg::String message;    // message to publish of String type
  // the line above can be written "auto message = std_msgs::msg::String();"
  message.data = "Hello, world! " + std::to_string(counter_++);   // the data of the message
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());    // printout of the message
  pub_->publish(message);   // publish method
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyPublisher>("my_publisher_node", 0);    // created a node of our class passing the name of
                                                                        //  the node and the counter init value
  rclcpp::spin(node);
  // it's possibile to define the two rows above using just one
  // rclcpp::spin(std::make_shared<MyPublisher>())

  rclcpp::shutdown();
  return 0;
}