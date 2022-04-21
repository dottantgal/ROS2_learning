/**
 * @file simple_action_client.cpp
 *
 * @brief A basic ROS2 action client node that sends as goal the number of string concatenation
 *        the action server should perform. 
 *        The server will send back feedbacks and the final result
 *
 * @author Antonio Mauro Galiano
 * Contact: https://www.linkedin.com/in/antoniomaurogaliano/
 *
 */


#include "custom_action/action/concatenate.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"


using Concatenate = custom_action::action::Concatenate;

// The callback for the feedbacks.
// The server sends back the partial sequence of the string concatenation
void FeedbackCallback(
  rclcpp_action::ClientGoalHandle<Concatenate>::SharedPtr,
  const std::shared_ptr<const Concatenate::Feedback> feedback,
  rclcpp::Logger logger)
{
  RCLCPP_INFO(logger,
    "Feedback received: %s",
    feedback->partial_concatenation.c_str());
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto client_node = rclcpp::Node::make_shared("simple_action_client");
  // action client definition
  // be sure to set the same action name used for the server
  auto action_client = rclcpp_action::create_client<Concatenate>(client_node, "concatenation");

  // waiting for the server online response
  if (!action_client->wait_for_action_server(std::chrono::seconds(10)))
  {
    RCLCPP_ERROR(client_node->get_logger(), "!!ATTENTION!! Action server not available");
    return 1;
  }

  // creation of the goal message
  auto goal_msg = Concatenate::Goal();
  goal_msg.num_concatenations = 9;  // the number of concatenation to perform

  RCLCPP_INFO(client_node->get_logger(), "Sending goal");
  // ask server to achieve some goal and wait until it's accepted
  // here some options such as feedbacks are set
  auto send_goal_options = rclcpp_action::Client<Concatenate>::SendGoalOptions();
  // binding to make the logger available in the callback
  // otherwise is necessary to declare the node handle as global
  send_goal_options.feedback_callback = std::bind(
    FeedbackCallback, std::placeholders::_1, std::placeholders::_2, client_node->get_logger());
  
  // the goal is sent and the server answers, it's necessary a SUCCESS to continue the communication 
  auto goal_handle_future = action_client->async_send_goal(goal_msg, send_goal_options);
  if (rclcpp::spin_until_future_complete(client_node, goal_handle_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(client_node->get_logger(), "send goal call failed :(");
    return 1;
  }

  // the server elaborates the goal and decide to reject or not the goal
  rclcpp_action::ClientGoalHandle<Concatenate>::SharedPtr goal_handle = goal_handle_future.get();
  if (!goal_handle)
  {
    RCLCPP_ERROR(client_node->get_logger(), "Goal was rejected by server");
    return 1;
  }

  // wait for the server to be done with the goal
  auto result_future = action_client->async_get_result(goal_handle);
  RCLCPP_INFO(client_node->get_logger(), "Waiting for result");
  if (rclcpp::spin_until_future_complete(client_node, result_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(client_node->get_logger(), "get result call failed :(");
    return 1;
  }

  // the clients gets the final result from the server
  rclcpp_action::ClientGoalHandle<Concatenate>::WrappedResult wrapped_result = result_future.get();
  switch (wrapped_result.code)
  {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(client_node->get_logger(), "Goal was aborted");
      return 1;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(client_node->get_logger(), "Goal was canceled");
      return 1;
    default:
      RCLCPP_ERROR(client_node->get_logger(), "Unknown result code");
      return 1;
  }

  // printout the result
  RCLCPP_INFO(client_node->get_logger(), "concatenation received");
  RCLCPP_INFO(client_node->get_logger(), "%s",  wrapped_result.result->final_concatenation.c_str());

  rclcpp::shutdown();
  return 0;
}