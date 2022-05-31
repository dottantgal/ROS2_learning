/**
 * @file client_node.cpp
 *
 * @brief A basic ROS2 service client node that asks the user to input two strings
 *        and gets back a capitalized full string from the server service.
 *        It's necessary to use the custom message defined in the external
 *        package "custom_msg_and_srv" 
 *
 * @author Antonio Mauro Galiano
 * Contact: https://www.linkedin.com/in/antoniomaurogaliano/
 *
 */


#include "custom_msg_and_srv/srv/capital_full_name.hpp" // custom service from another package
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;   // namespace to rappresent literal time period

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // the user inserts name and surname
  std::string name = "";
  std::string surname = "";
  std::cout << "Insert the name -> ";
  std::cin >> name;
  std::cout << "Insert the surname -> ";
  std::cin >> surname;

  // declaration of the client node
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("client_node");
  
  // declaration of the service client of type "create_cap_full_name"
  // using the custom srv "CapitalFullName"
  // It's used the method "create_client" of the class rclcpp::Node
  // The method is called by the arrow operator cause node is a shared pointer
  rclcpp::Client<custom_msg_and_srv::srv::CapitalFullName>::SharedPtr client =
      node->create_client<custom_msg_and_srv::srv::CapitalFullName>("create_cap_full_name");

  // can be declared as
  // std::shared_ptr<custom_msg_and_srv::srv::CapitalFullName::Request> request;
  auto request =
      std::make_shared<custom_msg_and_srv::srv::CapitalFullName::Request>();

  // fill in the request
  request->name = name;
  request->surname = surname;
  // set a literal time period to check the server availability
  std::chrono::seconds myPause = 1s;

  // check if the server is online and everything goes well otherwise printout
  while (!client->wait_for_service(1s))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                   "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "SERVICE NOT AVAILABLE, waiting again...");
  }

  // result type is rclcpp::Client<custom_msg_and_srv::srv::CapitalFullName>::SharedFuture
  // SharedFuture accesses the result of asynchronous operations, 
  // so multiple threads are allowed to wait for the same shared state
  auto result = client->async_send_request(request);
  
  // spins waiting for the SUCCESS result
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
      "Capitalized full name: %s", result.get()->capitalfullname.c_str());
  } else 
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service create_cap_full_name");
  }

  rclcpp::shutdown();
  return 0;
}
