/**
 * @file tf2_publisher.cpp
 *
 * @brief ROS2 node to publish TF2 transformation with run time adjustments through
 *        parameters setting
 *
 * @author Antonio Mauro Galiano
 * Contact: https://www.linkedin.com/in/antoniomaurogaliano/
 *
 */


#include <dynamic_tf2_publisher/tf2_publisher.h>


Tf2Publisher::Tf2Publisher() :  Node("tf2_dynamic_pub_node")
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Starting the dynamic TF2 publisher node");
  tfBroadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  declareParams();
  this->get_parameter("loop_rate", loopRate_);
  timer_ = this->create_wall_timer(
            std::chrono::milliseconds(loopRate_),
            std::bind(&Tf2Publisher::TimerCallback,this));
}


Tf2Publisher::~Tf2Publisher()
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Stopped the dynamic TF2 publisher node");
  tfBroadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}


void Tf2Publisher::declareParams()
{
  this->declare_parameter<std::string>("parent_frame", "");
  this->declare_parameter<std::string>("child_frame",  "");
  this->declare_parameter<float>("x", 0.0);
  this->declare_parameter<float>("y", 0.0);
  this->declare_parameter<float>("z", 0.0);
  this->declare_parameter<float>("roll", 0.0);
  this->declare_parameter<float>("pitch", 0.0);
  this->declare_parameter<float>("yaw", 0.0);

  this->declare_parameter<int>("loop_rate", 50);
}


void Tf2Publisher::TimerCallback()
{
  this->get_parameter("parent_frame", parentFrame_);
  this->get_parameter("child_frame", childFrame_);
  this->get_parameter("x", xPos_);
  this->get_parameter("y", yPos_);
  this->get_parameter("z", zPos_);
  this->get_parameter("roll", roll_);
  this->get_parameter("pitch", pitch_);
  this->get_parameter("yaw", yaw_);

  transform_.header.frame_id = parentFrame_;
  transform_.child_frame_id = childFrame_;
  transform_.transform.translation.x = xPos_;
  transform_.transform.translation.y = yPos_;
  transform_.transform.translation.z = zPos_;
  quat_.setRPY(roll_, pitch_, yaw_);
  transform_.transform.rotation.x = quat_.x();
  transform_.transform.rotation.y = quat_.y();
  transform_.transform.rotation.z = quat_.z();
  transform_.transform.rotation.w = quat_.w();

  try
  {
    rclcpp::Time now = this->get_clock()->now();
    transform_.header.stamp = now;
    tfBroadcaster_->sendTransform(transform_);
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Transformation error = " << ex.what());
  }
}
