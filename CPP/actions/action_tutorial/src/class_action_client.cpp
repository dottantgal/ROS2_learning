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


class ConcatenateActionClient : public rclcpp::Node
{
public:
  using Concatenate = custom_action::action::Concatenate;
  using GoalHandleConcatenate = rclcpp_action::ClientGoalHandle<Concatenate>;

  explicit ConcatenateActionClient(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
    : Node("class_action_client", node_options), goalDone_(false)
  {
    this->clientPtr_ = rclcpp_action::create_client<Concatenate>(
      this->get_node_base_interface(),
      this->get_node_graph_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "concatenation");

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&ConcatenateActionClient::SendGoal, this));
  }

  bool GoalDone() const;
  void SendGoal();

private:
  rclcpp_action::Client<Concatenate>::SharedPtr clientPtr_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool goalDone_;

  void FeedbackCallback(
    GoalHandleConcatenate::SharedPtr,
    const std::shared_ptr<const Concatenate::Feedback> feedback);
  void ResultCallback(const GoalHandleConcatenate::WrappedResult & result);
  // be sure to define the parameter as it's here
  // more info at the declaration
  void GoalResponseCallback(const GoalHandleConcatenate::SharedPtr &goalHandle);
};


bool ConcatenateActionClient::GoalDone() const
{
  return this->goalDone_;
}


void ConcatenateActionClient::SendGoal()
{
  using namespace std::placeholders;

  this->timer_->cancel();

  this->goalDone_ = false;

  if (!this->clientPtr_)
  {
    RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
  }

  if (!this->clientPtr_->wait_for_action_server(std::chrono::seconds(10)))
  {
    RCLCPP_ERROR(this->get_logger(), "!!ATTENTION!! Action server not available");
    this->goalDone_ = true;
    return;
  }

  auto goalMsg = Concatenate::Goal();
  goalMsg.num_concatenations = 9;

  RCLCPP_INFO(this->get_logger(), "Sending goal");

  auto send_goal_options = rclcpp_action::Client<Concatenate>::SendGoalOptions();
  send_goal_options.feedback_callback =
    std::bind(&ConcatenateActionClient::FeedbackCallback, this, _1, _2);
  send_goal_options.result_callback =
    std::bind(&ConcatenateActionClient::ResultCallback, this, _1);
  send_goal_options.goal_response_callback =
    std::bind(&ConcatenateActionClient::GoalResponseCallback, this, _1);
  auto goal_handle_future = this->clientPtr_->async_send_goal(goalMsg, send_goal_options);
}


void ConcatenateActionClient::FeedbackCallback(
  rclcpp_action::ClientGoalHandle<Concatenate>::SharedPtr,
  const std::shared_ptr<const Concatenate::Feedback> feedback)
{
  RCLCPP_INFO(this->get_logger(),
    "Feedback received: %s",
    feedback->partial_concatenation.c_str());
}


void ConcatenateActionClient::ResultCallback(const GoalHandleConcatenate::WrappedResult & result)
{
  this->goalDone_ = true;
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
  }

  RCLCPP_INFO(this->get_logger(), "Result received");
  for (auto number : result.result->final_concatenation)
  {
    RCLCPP_INFO(this->get_logger(), "%d", number);
  }
}

// defining the parameter directly as a GoalHandleConcatenate::SharedPtr goalHandle
// it's wrong for the send_goal_options.goal_response_callback
// so it doesnt compile
void ConcatenateActionClient::GoalResponseCallback(
  const GoalHandleConcatenate::SharedPtr &goalHandle)
  {
    if (!goalHandle)
    {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else
    {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto action_client = std::make_shared<ConcatenateActionClient>();

  while (!action_client->GoalDone())
  {
    rclcpp::spin_some(action_client);
  }

  rclcpp::shutdown();
  return 0;
}