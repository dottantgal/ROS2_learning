/**
 * @file service_node_class.cpp
 *
 * @brief A basic ROS2 service server node with class implementation that gets two 
 *        strings as request and answer with a capitalized full string as response.
 *        It's necessary to use the custom message defined in the external
 *        package "Custom msg and srv"
 *        To call the service from a terminal use on a single line:
 *        ros2 service call /create_cap_full_name 
 *        custom_msg_and_srv/srv/CapitalFullName "{name: x, surname: y}"
 *
 * @author Antonio Mauro Galiano
 * Contact: https://www.linkedin.com/in/antoniomaurogaliano/
 *
 */


#include "rclcpp/rclcpp.hpp"
#include "custom_msg_and_srv/srv/capital_full_name.hpp"
#include <boost/algorithm/string.hpp>


class MyServiceNode : public rclcpp::Node
{
private:
  rclcpp::Service<custom_msg_and_srv::srv::CapitalFullName>::SharedPtr service_;
  void ComposeFullName(const std::shared_ptr<custom_msg_and_srv::srv::CapitalFullName::Request> request,
        std::shared_ptr<custom_msg_and_srv::srv::CapitalFullName::Response> response);

public:
  MyServiceNode(std::string passedNodeName="VOID")
    : Node(passedNodeName)
  {
    RCLCPP_INFO(this->get_logger(), "I am ready to capitalize your full name");
    // like the subscriber class node it's needed the boost::bind to acces the member method 
    // with 2 placeholders to pass request and response to the callback
    service_ = this->create_service<custom_msg_and_srv::srv::CapitalFullName>("create_cap_full_name", 
      std::bind(&MyServiceNode::ComposeFullName, this, std::placeholders::_1, std::placeholders::_2 ));
  }
  
};

// method to handle the client request and give back a response
// the service gets the name and surname and responses with a capitalized full name
void MyServiceNode::ComposeFullName(const std::shared_ptr<custom_msg_and_srv::srv::CapitalFullName::Request> request,
        std::shared_ptr<custom_msg_and_srv::srv::CapitalFullName::Response> response)
{
  std::string fullName = request->name + " " + request->surname;
  std::string capitalFullName = boost::to_upper_copy<std::string>(fullName);
  response->capitalfullname = capitalFullName;    
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming name: %s" "\tIncoming surname: %s",
              request->name.c_str(), request->surname.c_str());
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending back the capitalize full name: [%s]", response->capitalfullname.c_str());
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyServiceNode>("service_node");
  rclcpp::spin(node);   // the service starts to wait and manage requests
  rclcpp::shutdown();
}
