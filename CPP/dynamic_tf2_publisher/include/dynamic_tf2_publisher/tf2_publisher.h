/**
 * @file tf2_publisher.h
 *
 * @brief Header file for tf2_publisher.cpp
 *
 * @author Antonio Mauro Galiano
 * Contact: https://www.linkedin.com/in/antoniomaurogaliano/
 *
 */


#ifndef TF2_PUBLISHER_H_
#define TF2_PUBLISHER_H_


#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"


class Tf2Publisher : public rclcpp::Node
{

public:
  Tf2Publisher();
  ~Tf2Publisher();

private:
  std::string parentFrame_, childFrame_;
  float xPos_, yPos_, zPos_, roll_, pitch_, yaw_;
  int loopRate_;
  tf2::Quaternion quat_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;
  geometry_msgs::msg::TransformStamped transform_;

  rclcpp::TimerBase::SharedPtr timer_;

  void declareParams();
  void TimerCallback();

};

#endif
