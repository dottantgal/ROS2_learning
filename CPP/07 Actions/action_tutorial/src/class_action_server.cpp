/**
 * @file class_action_server.cpp
 *
 * @brief A class defined ROS2 action server node that concatenates a string 
 *         based on the number of string concatenation sent by a client within a goal request
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

    this->actionServer_ = rclcpp_action::create_server<Concatenate>(
      this->get_node_base_interface(),
      this->get_node_clock_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "concatenation",
      std::bind(&ConcatenateActionServer::HandleGoal, this, _1, _2),
      std::bind(&ConcatenateActionServer::HandleCancel, this, _1),
      std::bind(&ConcatenateActionServer::HandleAccepted, this, _1));
  }

private:
  rclcpp_action::Server<Concatenate>::SharedPtr actionServer_;
  rclcpp_action::GoalResponse HandleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Concatenate::Goal> goal);
  rclcpp_action::CancelResponse HandleCancel(
    const std::shared_ptr<GoalHandleConcatenate> goalHandle);
  void execute(const std::shared_ptr<GoalHandleConcatenate> goalHandle);
  void HandleAccepted(const std::shared_ptr<ConcatenateActionServer::GoalHandleConcatenate> goalHandle);
};


rclcpp_action::GoalResponse ConcatenateActionServer::HandleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Concatenate::Goal> goal)
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


rclcpp_action::CancelResponse ConcatenateActionServer::HandleCancel(
    const std::shared_ptr<GoalHandleConcatenate> goalHandle)
{
  RCLCPP_INFO(rclcpp::get_logger("server"), "Got request to cancel goal");
  (void)goalHandle;
  return rclcpp_action::CancelResponse::ACCEPT;
}


void ConcatenateActionServer::execute(
  const std::shared_ptr<GoalHandleConcatenate> goalHandle)
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


void ConcatenateActionServer::HandleAccepted(
  const std::shared_ptr<GoalHandleConcatenate> goal_handle)
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