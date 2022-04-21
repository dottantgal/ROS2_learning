/**
 * @file class_action_client.cpp
 *
 * @brief A class defined ROS2 action client node that sends as goal the number of string 
 *        concatenation the action server should perform. 
 *        The server will send back feedbacks and the final result
 *
 * @author Antonio Mauro Galiano
 * Contact: https://www.linkedin.com/in/antoniomaurogaliano/
 *
 */


#include "custom_action/action/concatenate.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"


class ConcatenateActionServer : public rclcpp::Node
{
public:
  using Concatenate = custom_action::action::Concatenate;
  using GoalHandleConcatenate = rclcpp_action::ServerGoalHandle<Concatenate>;

  explicit ConcatenateActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("class_action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Concatenate>(
      this->get_node_base_interface(),
      this->get_node_clock_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "concatenation",
      std::bind(&ConcatenateActionServer::handle_goal, this, _1, _2),
      std::bind(&ConcatenateActionServer::handle_cancel, this, _1),
      std::bind(&ConcatenateActionServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<Concatenate>::SharedPtr action_server_;
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Concatenate::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleConcatenate> goal_handle);
  void execute(const std::shared_ptr<GoalHandleConcatenate> goal_handle);
  void handle_accepted(const std::shared_ptr<ConcatenateActionServer::GoalHandleConcatenate> goal_handle);
};


rclcpp_action::GoalResponse ConcatenateActionServer::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ConcatenateActionServer::Concatenate::Goal> goal)
{
  RCLCPP_INFO(rclcpp::get_logger("server"),
              "Got goal request with %d string concatenations",
              goal->num_concatenations);
  (void)uuid;
  // conditional to reject numbers of concatenations
  if ((goal->num_concatenations > 10) && (goal->num_concatenations < 2)) 
  {
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}


rclcpp_action::CancelResponse ConcatenateActionServer::handle_cancel(
    const std::shared_ptr<ConcatenateActionServer::GoalHandleConcatenate> goal_handle)
{
  RCLCPP_INFO(rclcpp::get_logger("server"), "Got request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}


void ConcatenateActionServer::execute(
  const std::shared_ptr<ConcatenateActionServer::GoalHandleConcatenate> goal_handle)
{
  RCLCPP_INFO(rclcpp::get_logger("server"), "Executing the concatenation");
  rclcpp::Rate loop_rate(1);
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<Concatenate::Feedback>();
  std::string myString = "HELLOWORLD";
  auto &concatenation = feedback->partial_concatenation;
  concatenation = myString;
  concatenation = concatenation + " " + myString;
  auto result = std::make_shared<Concatenate::Result>();

  for (int i = 1; (i < goal->num_concatenations) && rclcpp::ok(); ++i)
  {
    // check if there is a cancel request
    if (goal_handle->is_canceling())
    {
      result->final_concatenation = concatenation;
      goal_handle->canceled(result);
      RCLCPP_INFO(rclcpp::get_logger("server"), "Goal Canceled");
      return;
    }
    // update the final concatenation
    concatenation = concatenation + " " + myString;
    // update and publish feedback of the partial concatenation
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(rclcpp::get_logger("server"), "Publish Feedback");

    loop_rate.sleep();
  }

  // check if goal is done
  if (rclcpp::ok())
  {
    result->final_concatenation = concatenation;
    goal_handle->succeed(result);
    RCLCPP_INFO(rclcpp::get_logger("server"), "Goal Succeeded");
  }
}


void ConcatenateActionServer::handle_accepted(
  const std::shared_ptr<ConcatenateActionServer::GoalHandleConcatenate> goal_handle)
{
  using namespace std::placeholders;
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&ConcatenateActionServer::execute, this, _1), goal_handle}.detach();
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<ConcatenateActionServer>();

  rclcpp::spin(action_server);
  rclcpp::shutdown();
  return 0;
}