/**
 * @file simple_action_server.cpp
 *
 * @brief A basic ROS2 action server node that concatenates a string 
 *         based on the number of string concatenation sent by a client within a goal request
 *
 * @author Antonio Mauro Galiano
 * Contact: https://www.linkedin.com/in/antoniomaurogaliano/
 *
 */


#include "custom_action/action/concatenate.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"


using Concatenate = custom_action::action::Concatenate;
using GoalHandleConcatenate = rclcpp_action::ServerGoalHandle<Concatenate>;


// response to the client goal request
rclcpp_action::GoalResponse
  HandleGoal(const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const Concatenate::Goal> goal) 
{
  RCLCPP_INFO(rclcpp::get_logger("server"),
              "Got goal request with %d string concatenations",
              goal->num_concatenations);
  // rclcpp_action::GoalUUID is an array type
  // is it the id of the goal??? Its not so clear
  (void)uuid;
  
  // conditional to reject numbers of concatenations
  if ((goal->num_concatenations > 10) && (goal->num_concatenations < 2)) 
  {
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// response to cancel requests from the client
rclcpp_action::CancelResponse HandleCancel(const std::shared_ptr<GoalHandleConcatenate> goalHandle) 
{
  RCLCPP_INFO(rclcpp::get_logger("server"), "Got request to cancel goal");
  (void)goalHandle;
  return rclcpp_action::CancelResponse::ACCEPT;
}


// the goal is executed
void execute(const std::shared_ptr<GoalHandleConcatenate> goalHandle) 
{
  RCLCPP_INFO(rclcpp::get_logger("server"), "Executing the concatenation");
  rclcpp::Rate loop_rate(1);
  const auto goal = goalHandle->get_goal();
  auto feedback = std::make_shared<Concatenate::Feedback>();
  std::string myString = "HELLOWORLD";
  auto &concatenation = feedback->partial_concatenation;
  concatenation = myString;
  concatenation = concatenation + " " + myString;
  auto result = std::make_shared<Concatenate::Result>();

  for (int i = 1; (i < goal->num_concatenations) && rclcpp::ok(); ++i)
  {
    // check if there is a cancel request
    if (goalHandle->is_canceling())
    {
      result->final_concatenation = concatenation;
      goalHandle->canceled(result);
      RCLCPP_INFO(rclcpp::get_logger("server"), "Goal Canceled");
      return;
    }
    // update the final concatenation
    concatenation = concatenation + " " + myString;
    // update and publish feedback of the partial concatenation
    goalHandle->publish_feedback(feedback);
    RCLCPP_INFO(rclcpp::get_logger("server"), "Publish Feedback");

    loop_rate.sleep();
  }

  // check if goal is done
  if (rclcpp::ok())
  {
    result->final_concatenation = concatenation;
    goalHandle->succeed(result);
    RCLCPP_INFO(rclcpp::get_logger("server"), "Goal Succeeded");
  }
}

void HandleAccepted(const std::shared_ptr<GoalHandleConcatenate> goalHandle)
{
  // this needs to return quickly to avoid blocking the executor
  // so spin up a new thread
  std::thread{execute, goalHandle}.detach();
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("simple_action_server");

  // create an action server with three callbacks
  // 'handle_goal' and 'handle_cancel' are called by the Executor
  // (rclcpp::spin) 'execute' is called whenever 'handle_goal' returns by
  // accepting a goal
  // Calls to 'execute' are made in an available thread from a pool of four.
  auto action_server = rclcpp_action::create_server<Concatenate>(
      node, "concatenation", HandleGoal, HandleCancel, HandleAccepted);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}